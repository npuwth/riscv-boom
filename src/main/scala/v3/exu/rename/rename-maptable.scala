//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename Map Table
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.exu

import chisel3._
import chisel3.util._
import boom.v3.common._
import boom.v3.util._
import org.chipsalliance.cde.config.Parameters

class MapReq(val lregSz: Int) extends Bundle
{
  val lrs1 = UInt(lregSz.W)
  val lrs2 = UInt(lregSz.W)
  val lrs3 = UInt(lregSz.W)
  val ldst = UInt(lregSz.W)
} // maptable查询请求

class MapResp(val pregSz: Int) extends Bundle
{
  val prs1 = UInt(pregSz.W)
  val prs2 = UInt(pregSz.W)
  val prs3 = UInt(pregSz.W)
  val stale_pdst = UInt(pregSz.W)
} // maptable查询结果

class RemapReq(val lregSz: Int, val pregSz: Int) extends Bundle
{
  val ldst = UInt(lregSz.W)
  val pdst = UInt(pregSz.W)
  val valid = Bool()
} // maptable更新请求

// ------------------------------------------------------------//
// 以下开始定义MapTable模块，即 RenameMapTable
// （在rename-stage.scala中被实例化）
// ------------------------------------------------------------//

class RenameMapTable(
  val plWidth: Int,
  val numLregs: Int,
  val numPregs: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    // Logical sources -> physical sources.
    val map_reqs    = Input(Vec(plWidth, new MapReq(lregSz)))
    val map_resps   = Output(Vec(plWidth, new MapResp(pregSz)))
    // 1. 读取maptable的映射关系

    // Remapping an ldst to a newly allocated pdst?
    val remap_reqs  = Input(Vec(plWidth, new RemapReq(lregSz, pregSz)))
    // 2. 对maptable进行修改更新

    // Dispatching branches: need to take snapshots of table state.
    val ren_br_tags = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))
    // 3. 遇到分支指令进行snapshot

    // Signals for restoring state following misspeculation.
    val brupdate      = Input(new BrUpdateInfo)
    // 4. 对错误分支预测进行恢复

    val rollback    = Input(Bool())
  })

  // The map table register array and its branch snapshots.
  val map_table = RegInit(VecInit(Seq.fill(numLregs){0.U(pregSz.W)}))
  // 共有numLregs项，每项存放映射的物理寄存器号（初始化为0）
  val br_snapshots = Reg(Vec(maxBrCount, Vec(numLregs, UInt(pregSz.W))))

  // The intermediate states of the map table following modification by each pipeline slot.
  val remap_table = Wire(Vec(plWidth+1, Vec(numLregs, UInt(pregSz.W))))
  // 表示一系列中间状态，看成二维数组，索引范围0~plwidth
  // 0表示没有经过任何uop的原始maptable，i表示经过i个uop后的maptable
  // 所以总共有plWidth+1个，其更新方式详见下面scanLeft

  // Uops requesting changes to the map table.
  val remap_pdsts = io.remap_reqs map (_.pdst)
  val remap_ldsts_oh = io.remap_reqs map (req => UIntToOH(req.ldst) & Fill(numLregs, req.valid.asUInt))
  //ldst独热码，若不valid则为全0
  //remap_ldsts_oh是宽度为plWidth的数组，每个元素宽度为numLregs

  // Figure out the new mappings seen by each pipeline slot.
  for (i <- 0 until numLregs) { // 分别处理每一个逻辑寄存器
    if (i == 0 && !float) {
      for (j <- 0 until plWidth+1) {
        remap_table(j)(i) := 0.U // 0号逻辑寄存器始终映射到0号物理寄存器
      }
    } else {
      val remapped_row = (remap_ldsts_oh.map(ldst => ldst(i)) zip remap_pdsts)
        // ldst => ldst(i) 表示取出独热码中对应的第i位，看是不是1
        .scanLeft(map_table(i)) {case (pdst, (ldst, new_pdst)) => Mux(ldst, new_pdst, pdst)}
      // Mux：如果ldst为1，选择new_pdst，否则选择旧pdst
      // scanLeft从左向右计算，构成级联Mux，考虑每一个slot：
      //   maptable(i)  new_pdst(slot0)
      //            \    /
      //             Mux  <-- ldst
      //              |
      //             pdst   new_pdst(slot1)
      //                \    /
      //                 Mux  <-- ldst
      //                  |
      //                 pdst   new_pdst(slot2)
      //                      ……
      // 最后保留的是最新pdst，但产生的一系列pdst中间状态也会被记录，保存为br_snapshots（如果是分支）

      // scanLeft example:
      //   (1,2,3,4,5).scanLeft(5) {_ + _}
      //   result: 5,6,8,11,15,20 （即5,5+1,5+1+2,……）

      for (j <- 0 until plWidth+1) {
        remap_table(j)(i) := remapped_row(j)
      }
    }
  }

  // Create snapshots of new mappings.
  for (i <- 0 until plWidth) {
    when (io.ren_br_tags(i).valid) { // 是否为分支指令
      br_snapshots(io.ren_br_tags(i).bits) := remap_table(i+1)
      // 保存经过该分支指令后的maptable中间状态至br_snapshots
      // 为什么是后而不是前？因为分支指令可能改变maptable，如jal
    }
  }

  // 更新maptable
  when (io.brupdate.b2.mispredict) { // 发生mispredict
    // Restore the map table to a branch snapshot.
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
    // 恢复至相应br_tag的snapshot，消除错误路径的影响
  } .otherwise {
    // Update mappings.
    map_table := remap_table(plWidth) // 正常更新为经过所有slot后的结果
  }

  // Read out mappings.
  for (i <- 0 until plWidth) {
    io.map_resps(i).prs1       := (0 until i).foldLeft(map_table(io.map_reqs(i).lrs1)) ((p,k) =>
      Mux(bypass.B && io.remap_reqs(k).valid && io.remap_reqs(k).ldst === io.map_reqs(i).lrs1, io.remap_reqs(k).pdst, p))
    io.map_resps(i).prs2       := (0 until i).foldLeft(map_table(io.map_reqs(i).lrs2)) ((p,k) =>
      Mux(bypass.B && io.remap_reqs(k).valid && io.remap_reqs(k).ldst === io.map_reqs(i).lrs2, io.remap_reqs(k).pdst, p))
    io.map_resps(i).prs3       := (0 until i).foldLeft(map_table(io.map_reqs(i).lrs3)) ((p,k) =>
      Mux(bypass.B && io.remap_reqs(k).valid && io.remap_reqs(k).ldst === io.map_reqs(i).lrs3, io.remap_reqs(k).pdst, p))
    io.map_resps(i).stale_pdst := (0 until i).foldLeft(map_table(io.map_reqs(i).ldst)) ((p,k) =>
      Mux(bypass.B && io.remap_reqs(k).valid && io.remap_reqs(k).ldst === io.map_reqs(i).ldst, io.remap_reqs(k).pdst, p))
    // p是maptable(lrs1)，即直接读出的pdst
    // bypass在外面的RenameStage中统一处理，此处bypass.B始终为false
    // 也就是说，此处就是直接读取maptable的值，不考虑bypass

    if (!float) io.map_resps(i).prs3 := DontCare
  }

  // Don't flag the creation of duplicate 'p0' mappings during rollback.
  // These cases may occur soon after reset, as all maptable entries are initialized to 'p0'.
  io.remap_reqs map (req => (req.pdst, req.valid)) foreach {case (p,r) =>
    assert (!r || !map_table.contains(p) || p === 0.U && io.rollback, "[maptable] Trying to write a duplicate mapping.")}
}
