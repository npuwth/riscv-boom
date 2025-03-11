//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename FreeList
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.exu

import chisel3._
import chisel3.util._
import boom.v3.common._
import boom.v3.util._
import org.chipsalliance.cde.config.Parameters

// ------------------------------------------------------------//
// 以下开始定义FreeList模块，即 RenameFreeList
// （在rename-stage.scala中被实例化）
// ------------------------------------------------------------//

class RenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Bool()))
    val alloc_pregs   = Output(Vec(plWidth, Valid(UInt(pregSz.W))))
    // 1. 分配新的物理寄存器号

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W))))
    // 2. ROB释放的物理寄存器号

    // Branch info for starting new allocation lists.
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))
    // 3. 遇到分支指令进行snapshot

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)
    // 4. 对错误分支预测进行恢复

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  // bit-vector，宽度为numPregs，每个bit代表对应物理寄存器是否空闲（1表示空闲）
  // 除0号寄存器外，均初始化为0
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W))) // snapshots
  // br_alloc_lists表示这条br指令之后的所有指令分配的物理寄存器，需要不断维护
  // （注意：与maptable中snapshot的含义和用法有些不同）

  // Select pregs from the free list.
  val sels = SelectFirstN(free_list, plWidth)
  // 借助PriorityEncoderOH实现，for循环，循环plWidth次
  // 返回的sels是plWidth次分配构成的序列，每个元素表示每次分配了哪个物理寄存器
  //   eg. 4次分配：10000000,00100000,00001000,00000001
  // 不是每条指令都需要物理寄存器，但从硬件实现角度，只能每次都选出plWidth个，不需要可以不用
  val sel_fire  = Wire(Vec(plWidth, Bool()))
  // sel_fire表示是否需要使用（由于寄存器分配使用两拍实现，实际为r_sel的写使能信号，详见下）

  // Allocations seen by branches in each pipeline slot.
  val allocs = io.alloc_pregs map (a => UIntToOH(a.bits))
  // alloc_pregs是freelist物理寄存器分配的输出结果，转为独热码
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r) }
  // (a,r)中a是分配的物理寄存器号独热码，r是请求有效
  // 与maptable中类似，通过scanRight得到当前周期分配的所有物理寄存器，eg. 10101001
  // 以及一系列分配的中间状态（表示该br指令之后的所有指令分配的物理寄存器）
  // scanRight从右向左计算，alloc_masks(0)是最终结果，alloc_masks(plWidth)是全0

  // Masks that modify the freelist array.
  val sel_mask = (sels zip sel_fire) map { case (s,f) => s & Fill(n,f) } reduce(_|_)
  // 将sels和sel_fire（是否需要使用）进行取与，再进行reduce
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  // br_alloc_lists表示这条br指令之后的所有指令分配的物理寄存器（即错误路径产生的影响）
  // 因此，在发生mispredict后，用其进行dealloc（释放，即和当前freelist按位或）
  val dealloc_mask = io.dealloc_pregs.map(d => UIntToOH(d.bits)(numPregs-1,0) & Fill(n,d.valid)).reduce(_|_) | br_deallocs
  // 另一种dealloc来自Commit Stage（stale_pdst），和上面的br_dealloc进行按位或

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt // 表示是否为分支指令

  // Create branch allocation lists.
  // 不断维护br_alloc_lists
  for (i <- 0 until maxBrCount) { // 分别处理每一个br_tag
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    // 先转独热码再提取出第i位，构成长度plWidth的数组，再与br_slots进行按位与
    // 表示：plWidth个slot中某指令是否为分支且其br_tag为i
    val new_list = list_req.orR
    // 进行缩减或，若不为0，说明存在指令为分支且其br_tag为i
    // 这种情况下，br_alloc_lists是新分配
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),  // 新分配
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0)) // 正常更新
    // 两种更新情况：
    //     （1）新分配：这个br_tag是新分配的，只考虑当前周期这条br指令之后的所有指令分配的物理寄存器，
    //         根据list_req从上述alloc_masks（中间状态）中选择对应的项
    //     （2）正常更新：将这周期释放的br_deallocs和新分配的alloc_masks信息更新进br_alloc_lists
  }

  // Update the free list.
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))
  // sel_mask是分配；dealloc_mask是释放；0号始终不可用

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val can_sel = sels(w).orR
    // 缩减或，判断sels(w)是否全0，是则分配不了空闲物理寄存器
    val r_valid = RegInit(false.B)
    val r_sel   = RegEnable(OHToUInt(sels(w)), sel_fire(w))
    // 定义流水线寄存器
    // 由于选择空闲物理寄存器的逻辑（SelectFirstN）较为复杂，
    // 选出后还需传递至Rename Stage中更新maptable和busytable，
    // 因此用流水线寄存器寄存一拍，当前周期选出的寄存器将在下一周期使用

    r_valid := r_valid && !io.reqs(w) || can_sel
    // 上一拍就有效且当前拍不用掉 || 能够分配到新的
    sel_fire(w) := (!r_valid || io.reqs(w)) && can_sel
    // r_sel写使能：r_valid无效或请求有效，并且can_sel（分配成功）
    //     （1）r_valid无效：说明当前没取好，要取
    //     （2）请求有效：说明取好了但当前拍要用掉，还得取

    io.alloc_pregs(w).bits  := r_sel
    io.alloc_pregs(w).valid := r_valid
    // 寄存一拍的分配的物理寄存器号作为freelist的最终输出
  }

  io.debug.freelist := free_list | io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n,p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}
