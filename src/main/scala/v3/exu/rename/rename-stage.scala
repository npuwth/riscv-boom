//******************************************************************************
// Copyright (c) 2012 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Datapath: Rename Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Supports 1-cycle and 2-cycle latencies. (aka, passthrough versus registers between ren1 and ren2).
//    - ren1: read the map tables and allocate a new physical register from the freelist.
//    - ren2: read the busy table for the physical operands.
//
// Ren1 data is provided as an output to be fed directly into the ROB.

package boom.v3.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._
import boom.v3.util._

/**
 * IO bundle to interface with the Register Rename logic
 *
 * @param plWidth pipeline width
 * @param numIntPregs number of int physical registers
 * @param numFpPregs number of FP physical registers
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStageIO(
  val plWidth: Int,     // 流水线宽度
  val numPhysRegs: Int, // 物理寄存器数目
  val numWbPorts: Int)  // 写回端口数目
  (implicit p: Parameters) extends BoomBundle


/**
 * IO bundle to debug the rename stage
 */
class DebugRenameStageIO(val numPhysRegs: Int)(implicit p: Parameters) extends BoomBundle
{
  val freelist  = Bits(numPhysRegs.W)
  val isprlist  = Bits(numPhysRegs.W)
  val busytable = UInt(numPhysRegs.W)
}

// 抽象类，有两种具体实现：RenameStage和PredRenameStage
// 前者为通用正常重命名，后者用于sfb优化
// 抽象类中定义的，是两种重命名实现中的共有部分
abstract class AbstractRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val ren_stalls = Output(Vec(plWidth, Bool())) // 是否阻塞

    val kill = Input(Bool())

    val dec_fire  = Input(Vec(plWidth, Bool())) // will commit state updates
    val dec_uops  = Input(Vec(plWidth, new MicroOp()))
    // 来自前一流水级decode的MicroOp

    // physical specifiers available AND busy/ready status available.
    val ren2_mask = Vec(plWidth, Output(Bool())) // mask of valid instructions
    val ren2_uops = Vec(plWidth, Output(new MicroOp()))
    // 重命名stage2的输出，即最终输出

    // branch resolution (execute)
    val brupdate = Input(new BrUpdateInfo())
    // 来自Execute Stage的分支解决信息

    val dis_fire  = Input(Vec(coreWidth, Bool()))
    val dis_ready = Input(Bool())
    // 来自后一流水级dispatch的信息，是否阻塞

    // wakeup ports
    val wakeups = Flipped(Vec(numWbPorts, Valid(new ExeUnitResp(xLen))))
    // 来自Execute Stage的唤醒信号

    // commit stage
    val com_valids = Input(Vec(plWidth, Bool()))
    val com_uops = Input(Vec(plWidth, new MicroOp()))
    val rbk_valids = Input(Vec(plWidth, Bool()))
    val rollback = Input(Bool())
    // 来自Commit Stage的信息：正常提交/回滚

    val debug_rob_empty = Input(Bool())
    val debug = Output(new DebugRenameStageIO(numPhysRegs))
  })

  io.ren_stalls.foreach(_ := false.B)
  io.debug := DontCare

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp

  //-------------------------------------------------------------
  // Pipeline State & Wires
  // 分阶段声明一些信号：
  //     （1）wire类型表示一些中间结果
  //     （2）reg类型表示流水线寄存器
  // Stage 1
  val ren1_fire       = Wire(Vec(plWidth, Bool()))
  val ren1_uops       = Wire(Vec(plWidth, new MicroOp))


  // Stage 2
  val ren2_fire       = io.dis_fire
  val ren2_ready      = io.dis_ready
  val ren2_valids     = Wire(Vec(plWidth, Bool()))
  val ren2_uops       = Wire(Vec(plWidth, new MicroOp))
  val ren2_alloc_reqs = Wire(Vec(plWidth, Bool()))
  // 在stage 2进行物理寄存器的分配

  //-------------------------------------------------------------
  // pipeline registers

  for (w <- 0 until plWidth) {
    ren1_fire(w)          := io.dec_fire(w)
    ren1_uops(w)          := io.dec_uops(w)
  } // stage 1的fire和uop接收自decode

  for (w <- 0 until plWidth) {
    val r_valid  = RegInit(false.B)
    val r_uop    = Reg(new MicroOp)
    // 两阶段重命名的划分边界：流水线寄存器
    val next_uop = Wire(new MicroOp)
    // 下一cycle，流水线寄存器uop的值

    next_uop := r_uop // 默认等于当前cycle的uop

    when (io.kill) {
      r_valid := false.B // 复位为false
    } .elsewhen (ren2_ready) { // dispatch没有任何阻塞
      r_valid := ren1_fire(w)
      next_uop := ren1_uops(w)
      // 从stage 1接收新的uop作为下一uop
    } .otherwise {             // dispatch存在阻塞
      r_valid := r_valid && !ren2_fire(w) // clear bit if uop gets dispatched
      // 阻塞的uop（ren2_fire为false）无法发射，因此仍然有效（或仍然无效）
      // “不阻塞”的uop可以发射（还涉及uop间的顺序问题，详见core.scala）
      // 发射的uop的valid置零，防止重复dispatch
      next_uop := r_uop
      // 虽然可以发射部分uop至dispatch，但不会从stage 1接收新的uop
      // 需要等待现在的这些uop全部发射完（因为Rename需要严格按序）
    }

    r_uop := GetNewUopAndBrMask(BypassAllocations(next_uop, ren2_uops, ren2_alloc_reqs), io.brupdate)
    // next_uop（下一cycle的uop）并不会直接赋给r_uop（流水线寄存器）
    // 需要经过（1）bypass和（2）branch处理逻辑
    // 前者处理next_uop和上一周期ren2_uops的bypass关系（ren2_uops对maptable的更新正在进行）
    // 后者根据分支解决信息更新next_uop的br_mask：
    //     br_mask := uop.br_mask & ~brupdate.b1.resolve_mask

    ren2_valids(w) := r_valid
    ren2_uops(w)   := r_uop
    // 将流水线寄存器的值赋给ren2_uops，但这并不是最终的输出，
    // 在RenameStage和PredRenameStage中都将进行更具体的部分赋值（例如pdst）
  }

  //-------------------------------------------------------------
  // Outputs

  io.ren2_mask := ren2_valids


}

// ------------------------------------------------------------//
// 以下开始定义重命名模块（阶段），即 RenameStage
// 可以被配置为浮点/定点寄存器重命名
// （在core.scala中被实例化）
// ------------------------------------------------------------//

/**
 * Rename stage that connets the map table, free list, and busy table.
 * Can be used in both the FP pipeline and the normal execute pipeline.
 *
 * @param plWidth pipeline width
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int,
  float: Boolean)
(implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{
  val pregSz = log2Ceil(numPhysRegs)
  val rtype = if (float) RT_FLT else RT_FIX // 寄存器类型，浮点/定点

  //-------------------------------------------------------------
  // Helper Functions

  // 在RenameStage中存在两种bypass：
  //     （1）与同周期更早（老）uop的bypass
  //     （2）与上一周期uop的bypass
  // 后者是因为ren1的uop读取maptable时，ren2还未完成更新
  // 两种bypass通过下面的统一方法实现
  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    val bypassed_uop = Wire(new MicroOp)
    bypassed_uop := uop // 先赋值，再进行部分修改

    val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs1 }
    val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs2 }
    val bypass_hits_rs3 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs3 }
    val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.ldst }
    // zip作用：两个序列两两配对形成一个（x，y）序列
    // alloc_reqs表示older_uops是否申请物理寄存器（是否写回目的寄存器），为bool序列
    // case (r,a) => 后为发生bypass的条件：写回目的寄存器 && 目的寄存器号等于源寄存器号
    // 得到的bypass_hits_rs1：每一个older_uops的lrs1 bypass hit情况

    // 除3个源寄存器lrs1、lrs2、lrs3外，ldst也需要bypass，
    // 若bypass hit，则stale_pdst的值为older_uop新分配的pdst
    // stale_pdst记录替换掉的旧目的物理寄存器，在commit时释放

    val bypass_sel_rs1 = PriorityEncoderOH(bypass_hits_rs1.reverse).reverse
    val bypass_sel_rs2 = PriorityEncoderOH(bypass_hits_rs2.reverse).reverse
    val bypass_sel_rs3 = PriorityEncoderOH(bypass_hits_rs3.reverse).reverse
    val bypass_sel_dst = PriorityEncoderOH(bypass_hits_dst.reverse).reverse
    // PriorityEncoderOH产生独热码，只有最低的那个1保留
    // 考虑到bypass中有多个older_uop hit的情况，只有最新的那个保留

    val do_bypass_rs1 = bypass_hits_rs1.reduce(_||_)
    val do_bypass_rs2 = bypass_hits_rs2.reduce(_||_)
    val do_bypass_rs3 = bypass_hits_rs3.reduce(_||_)
    val do_bypass_dst = bypass_hits_dst.reduce(_||_)
    // 对bool序列进行规约（缩减或），表示是否存在bypass（1表示存在）

    val bypass_pdsts = older_uops.map(_.pdst) // older_uops的pdst序列

    when (do_bypass_rs1) { bypassed_uop.prs1       := Mux1H(bypass_sel_rs1, bypass_pdsts) }
    when (do_bypass_rs2) { bypassed_uop.prs2       := Mux1H(bypass_sel_rs2, bypass_pdsts) }
    when (do_bypass_rs3) { bypassed_uop.prs3       := Mux1H(bypass_sel_rs3, bypass_pdsts) }
    when (do_bypass_dst) { bypassed_uop.stale_pdst := Mux1H(bypass_sel_dst, bypass_pdsts) }
    // Mux1H：根据输入的独热码，从pdst序列中选择一个作为输出

    bypassed_uop.prs1_busy := uop.prs1_busy || do_bypass_rs1
    bypassed_uop.prs2_busy := uop.prs2_busy || do_bypass_rs2
    bypassed_uop.prs3_busy := uop.prs3_busy || do_bypass_rs3
    // 对uop源寄存器的busy状态也进行bypass：
    //     若发生bypass，则该源寄存器肯定处于busy状态

    if (!float) { // 定点指令用不到 lrs3
      bypassed_uop.prs3      := DontCare
      bypassed_uop.prs3_busy := false.B
    }

    bypassed_uop // 返回更新（经过bypass）后的uop
  }

  //-------------------------------------------------------------
  // Rename Structures
  // 下面对maptable、freelist、busytable三个子模块进行实例化
  val maptable = Module(new RenameMapTable(
    plWidth,
    32,
    numPhysRegs,
    false, // 由于在外部（rename-stage.scala）中进行bypass处理，模块内部不需要
    float))
  val freelist = Module(new RenameFreeList(
    plWidth,
    numPhysRegs,
    if (float) 32 else 31))
  val busytable = Module(new RenameBusyTable(
    plWidth,
    numPhysRegs,
    numWbPorts,
    false,
    float))
  // 浮点有32个逻辑寄存器；
  // 定点有31个（0号逻辑固定映射到0号物理，不用重命名）


  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))
  //Valid( )给interface增加一个valid位

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  // 一些控制信号产生逻辑
  for (w <- 0 until plWidth) {
    ren2_alloc_reqs(w)    := ren2_uops(w).ldst_val && ren2_uops(w).dst_rtype === rtype && ren2_fire(w)
    // ren2_uop需要写回dst && 类型相符 && 需要发射，则请求分配一个物理寄存器
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    com_valids(w)         := io.com_uops(w).ldst_val && io.com_uops(w).dst_rtype === rtype && io.com_valids(w)
    // com_uop需要写回dst && 类型相符 && commit有效
    rbk_valids(w)         := io.com_uops(w).ldst_val && io.com_uops(w).dst_rtype === rtype && io.rbk_valids(w)
    // com_uop需要写回dst && 类型相符 && rollback有效
    ren2_br_tags(w).bits  := ren2_uops(w).br_tag
  }

  //-------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs   = Wire(Vec(plWidth, new MapReq(lregSz)))
  val remap_reqs = Wire(Vec(plWidth, new RemapReq(lregSz, pregSz)))

  // Generate maptable requests.
  // 生成maptable请求：
  //     （1）map_req，读取maptable的请求
  //     （2）remap_req，更新maptable的请求
  // 后者又分为两种情况：
  //     （1）回滚，传入commit时的ldst和stale_pdst，将ldst映射回stale_pdst
  //     （2）正常分配，传入ren2时的ldst和pdst，将ldst映射为pdst
  for ((((ren1,ren2),com),w) <- (ren1_uops zip ren2_uops zip io.com_uops.reverse).zipWithIndex) {
    // 多个序列的连续嵌套zip
    // 进行回滚的时候，释放顺序是相反的，所以要reverse
    map_reqs(w).lrs1 := ren1.lrs1
    map_reqs(w).lrs2 := ren1.lrs2
    map_reqs(w).lrs3 := ren1.lrs3
    map_reqs(w).ldst := ren1.ldst

    remap_reqs(w).ldst := Mux(io.rollback, com.ldst      , ren2.ldst)
    remap_reqs(w).pdst := Mux(io.rollback, com.stale_pdst, ren2.pdst)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip remap_reqs map {
    case ((a,r),rr) => rr.valid := a || r}
  // remap_req是否有效：ren2分配请求有效 / commit回滚请求有效

  // Hook up inputs. maptable input连线
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.ren_br_tags := ren2_br_tags
  maptable.io.brupdate      := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  // 根据maptable的输出map_resps，更新ren1_uops
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)

    uop.prs1       := mappings.prs1
    uop.prs2       := mappings.prs2
    uop.prs3       := mappings.prs3 // only FP has 3rd operand
    uop.stale_pdst := mappings.stale_pdst
  }



  //-------------------------------------------------------------
  // Free List

  // Freelist inputs. freelist input连线
  freelist.io.reqs := ren2_alloc_reqs // ren2分配请求传至freelist
  freelist.io.dealloc_pregs zip com_valids zip rbk_valids map
  // 这里释放无所谓rbk的顺序，不用reverse
    {case ((d,c),r) => d.valid := c || r}
  freelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  // 两种情况：
  //     （1）回滚，释放pdst
  //     （2）正常提交，释放stale_pdst
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  assert (ren2_alloc_reqs zip freelist.io.alloc_pregs map {case (r,p) => !r || p.bits =/= 0.U} reduce (_&&_),
           "[rename-stage] A uop is trying to allocate the zero physical register.")
  // assert内表达式应为 1，只在仿真时起作用
  // “0号物理寄存器不能被freelist所分配”

  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val preg = freelist.io.alloc_pregs(w).bits // 分配的物理寄存器号
    uop.pdst := Mux(uop.ldst =/= 0.U || float.B, preg, 0.U)
    // 定点且逻辑寄存器号为0，固定映射到0号物理寄存器
  }

  //-------------------------------------------------------------
  // Busy Table
  // Busytable inputs. busytable input连线
  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  // 将新分配的物理寄存器置为busy
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pdst)
  // 来自Execute Stage的唤醒信息

  assert (!(io.wakeups.map(x => x.valid && x.bits.uop.dst_rtype =/= rtype).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val busy = busytable.io.busy_resps(w)

    uop.prs1_busy := uop.lrs1_rtype === rtype && busy.prs1_busy
    uop.prs2_busy := uop.lrs2_rtype === rtype && busy.prs2_busy
    uop.prs3_busy := uop.frs3_en && busy.prs3_busy // 浮点才有效

    val valid = ren2_valids(w)
    assert (!(valid && busy.prs1_busy && rtype === RT_FIX && uop.lrs1 === 0.U), "[rename] x0 is busy??")
    assert (!(valid && busy.prs2_busy && rtype === RT_FIX && uop.lrs2 === 0.U), "[rename] x0 is busy??")
    // “定点的0号寄存器不可能busy”
  }

  //-------------------------------------------------------------
  // Rename Stage Outputs

  for (w <- 0 until plWidth) {
    val can_allocate = freelist.io.alloc_pregs(w).valid

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).dst_rtype === rtype) && !can_allocate
    // 没有空闲物理寄存器了且需要分配，发生阻塞

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)
    // slice对序列进行切片，slice(0,w)：从0到w-1
    // 这里是对同周期uop的bypass，除0号uop外，其余uop需要对前面的0到w-1进行bypass

    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
    // 根据分支解决信息更新br_mask
  }

  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}

// ------------------------------------------------------------//
// 以下开始定义SFB重命名模块（阶段），即 PredRenameStage
// （在core.scala中被实例化）
// ------------------------------------------------------------//

// 以下与SFB优化相关，暂未研究（TODO）
class PredRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
  (implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{
  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    uop
  }

  ren2_alloc_reqs := DontCare

  val busy_table = RegInit(VecInit(0.U(ftqSz.W).asBools))
  val to_busy = WireInit(VecInit(0.U(ftqSz.W).asBools))
  val unbusy = WireInit(VecInit(0.U(ftqSz.W).asBools))

  val current_ftq_idx = Reg(UInt(log2Ceil(ftqSz).W))
  var next_ftq_idx = current_ftq_idx

  for (w <- 0 until plWidth) {
    io.ren2_uops(w) := ren2_uops(w)

    val is_sfb_br = ren2_uops(w).is_sfb_br && ren2_fire(w)
    val is_sfb_shadow = ren2_uops(w).is_sfb_shadow && ren2_fire(w)

    val ftq_idx = ren2_uops(w).ftq_idx
    when (is_sfb_br) {
      io.ren2_uops(w).pdst := ftq_idx
      to_busy(ftq_idx) := true.B
    }
    next_ftq_idx = Mux(is_sfb_br, ftq_idx, next_ftq_idx)

    when (is_sfb_shadow) {
      io.ren2_uops(w).ppred := next_ftq_idx
      io.ren2_uops(w).ppred_busy := (busy_table(next_ftq_idx) || to_busy(next_ftq_idx)) && !unbusy(next_ftq_idx)
    }
  }

  for (w <- 0 until numWbPorts) {
    when (io.wakeups(w).valid) {
      unbusy(io.wakeups(w).bits.uop.pdst) := true.B
    }
  }

  current_ftq_idx := next_ftq_idx

  busy_table := ((busy_table.asUInt | to_busy.asUInt) & ~unbusy.asUInt).asBools
}
