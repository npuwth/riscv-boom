//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.v3.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.Instructions32
import freechips.rocketchip.rocket.CustomInstructions._
import freechips.rocketchip.rocket.RVCExpander
import freechips.rocketchip.rocket.{CSR,Causes}
import freechips.rocketchip.util.{uintToBitPat,UIntIsOneOf}

import FUConstants._
import boom.v3.common._
import boom.v3.util._

// scalastyle:off
/**
 * Abstract trait giving defaults and other relevant values to different Decode constants/
 */
abstract trait DecodeConstants
  extends freechips.rocketchip.rocket.constants.ScalarOpConstants
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val xpr64 = Y // TODO inform this from xLen
  val DC2 = BitPat.dontCare(2) // Makes the listing below more readable
  // BitPat是BitPattern的缩写，在Chisel中用于进行bit pattern匹配
  // 上面的DC2定义了"b??"，表示匹配一个两位宽的信号，其具体值并不关心
  // 另一个例子，BitPat("b1??0")表示匹配信号：1xx0
  def decode_default: List[BitPat] =
  // 定义一些默认值
  // N: false, Y: true, X: dont care
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
              List(N, N, X, uopX    , IQT_INT, FU_X   , RT_X  , DC2    ,DC2    ,X, IS_X, X, X, X, X, N, M_X,   DC2, X, X, N, N, X, CSR.X)

  val table: Array[(BitPat, List[BitPat])]
  // 译码表被定义成由(Bit Pattern, List)构成的数组，前者用于匹配具体的RISCV指令，
  // 后者则是译码结果构成的List，List中每一项也都被声明为一个BitPat，详细定义在consts.scala中
  // 译码结果一共24项，包括是否浮点、uop类型、寄存器类型、是否amo、是否分支、是否ecall等等
  // 这里的译码结果主要是控制信号，不包括寄存器号、立即数
}
// scalastyle:on

/**
 * Decoded control signals
 */
class CtrlSigs extends Bundle
{
  val legal           = Bool()
  val fp_val          = Bool()
  val fp_single       = Bool()
  val uopc            = UInt(UOPC_SZ.W)
  val iq_type         = UInt(IQT_SZ.W)
  val fu_code         = UInt(FUC_SZ.W)
  val dst_type        = UInt(2.W)
  val rs1_type        = UInt(2.W)
  val rs2_type        = UInt(2.W)
  val frs3_en         = Bool()
  val imm_sel         = UInt(IS_X.getWidth.W)
  val uses_ldq        = Bool()
  val uses_stq        = Bool()
  val is_amo          = Bool()
  val is_fence        = Bool()
  val is_fencei       = Bool()
  val mem_cmd         = UInt(freechips.rocketchip.rocket.M_SZ.W)
  val wakeup_delay    = UInt(2.W)
  val bypassable      = Bool()
  val is_br           = Bool()
  val is_sys_pc2epc   = Bool()
  val inst_unique     = Bool()
  val flush_on_commit = Bool()
  val csr_cmd         = UInt(freechips.rocketchip.rocket.CSR.SZ.W)
  val rocc            = Bool() // 相比List[BitPat]多了这一项

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    // 该函数在下面的DecodeUnit中调用
    val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, XDecode.decode_default, table)
    // 复用rocket中的译码逻辑，传入：当前需要译码的指令、默认值、译码表
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, dst_type, rs1_type,
          rs2_type, frs3_en, imm_sel, uses_ldq, uses_stq, is_amo,
          is_fence, is_fencei, mem_cmd, wakeup_delay, bypassable,
          is_br, is_sys_pc2epc, inst_unique, flush_on_commit, csr_cmd)
      sigs zip decoder map {case(s,d) => s := d} // 对sigs中的信号一一赋值，即译码结果
      rocc := false.B // 表示协处理器指令，赋值为false
      this // 返回sigs
  }
}

// scalastyle:off
/**
 * Decode constants for RV32
 */
object X32Decode extends DecodeConstants
{
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//   |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  Instructions32.SLLI ->
              List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  Instructions32.SRLI ->
              List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  Instructions32.SRAI ->
              List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N)
  )
}

/**
 * Decode constants for RV64
 */
object X64Decode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LD      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LWU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  SD      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  SLLI    -> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLI    -> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAI    -> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDIW   -> List(Y, N, X, uopADDIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLLIW   -> List(Y, N, X, uopSLLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAIW   -> List(Y, N, X, uopSRAIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLIW   -> List(Y, N, X, uopSRLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDW    -> List(Y, N, X, uopADDW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SUBW    -> List(Y, N, X, uopSUBW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLLW    -> List(Y, N, X, uopSLLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAW    -> List(Y, N, X, uopSRAW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLW    -> List(Y, N, X, uopSRLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N)
  )
}

/**
 * Overall Decode constants
 */
object XDecode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LW      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LH      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LHU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LB      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LBU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),

  SW      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),
  SH      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),
  SB      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  LUI     -> List(Y, N, X, uopLUI  , IQT_INT, FU_ALU , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDI    -> List(Y, N, X, uopADDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ANDI    -> List(Y, N, X, uopANDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ORI     -> List(Y, N, X, uopORI  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  XORI    -> List(Y, N, X, uopXORI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTI    -> List(Y, N, X, uopSLTI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTIU   -> List(Y, N, X, uopSLTIU, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  SLL     -> List(Y, N, X, uopSLL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ADD     -> List(Y, N, X, uopADD  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SUB     -> List(Y, N, X, uopSUB  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLT     -> List(Y, N, X, uopSLT  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTU    -> List(Y, N, X, uopSLTU , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  AND     -> List(Y, N, X, uopAND  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  OR      -> List(Y, N, X, uopOR   , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  XOR     -> List(Y, N, X, uopXOR  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRA     -> List(Y, N, X, uopSRA  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRL     -> List(Y, N, X, uopSRL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  MUL     -> List(Y, N, X, uopMUL  , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULH    -> List(Y, N, X, uopMULH , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULHU   -> List(Y, N, X, uopMULHU, IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULHSU  -> List(Y, N, X, uopMULHSU,IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULW    -> List(Y, N, X, uopMULW , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  DIV     -> List(Y, N, X, uopDIV  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVU    -> List(Y, N, X, uopDIVU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REM     -> List(Y, N, X, uopREM  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMU    -> List(Y, N, X, uopREMU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVW    -> List(Y, N, X, uopDIVW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVUW   -> List(Y, N, X, uopDIVUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMW    -> List(Y, N, X, uopREMW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMUW   -> List(Y, N, X, uopREMUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  AUIPC   -> List(Y, N, X, uopAUIPC, IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N), // use BRU for the PC read
  JAL     -> List(Y, N, X, uopJAL  , IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_J, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N),
  JALR    -> List(Y, N, X, uopJALR , IQT_INT, FU_JMP , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N),
  BEQ     -> List(Y, N, X, uopBEQ  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BNE     -> List(Y, N, X, uopBNE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BGE     -> List(Y, N, X, uopBGE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BGEU    -> List(Y, N, X, uopBGEU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BLT     -> List(Y, N, X, uopBLT  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BLTU    -> List(Y, N, X, uopBLTU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),

  // I-type, the immediate12 holds the CSR register.
  CSRRW   -> List(Y, N, X, uopCSRRW, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W),
  CSRRS   -> List(Y, N, X, uopCSRRS, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S),
  CSRRC   -> List(Y, N, X, uopCSRRC, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C),

  CSRRWI  -> List(Y, N, X, uopCSRRWI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W),
  CSRRSI  -> List(Y, N, X, uopCSRRSI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S),
  CSRRCI  -> List(Y, N, X, uopCSRRCI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C),

  SFENCE_VMA->List(Y,N, X, uopSFENCE,IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N,M_SFENCE,0.U,N, N, N, Y, Y, CSR.N),
  ECALL   -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I),
  EBREAK  -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I),
  SRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),
  MRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),
  DRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),

  WFI     -> List(Y, N, X, uopWFI   ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),

  FENCE_I -> List(Y, N, X, uopNOP  , IQT_INT, FU_X   , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, Y, M_X  , 0.U, N, N, N, Y, Y, CSR.N),
  FENCE   -> List(Y, N, X, uopFENCE, IQT_INT, FU_MEM , RT_X  , RT_X  , RT_X  , N, IS_X, N, Y, N, Y, N, M_X  , 0.U, N, N, N, Y, Y, CSR.N), // TODO PERF make fence higher performance
                                                                                                                                                       // currently serializes pipeline

           //                                                                  frs3_en                           wakeup_delay
           //     is val inst?                                                 |  imm sel                        |   bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq                 |   |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq              |   |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo             |   |  |
           //     |  |  |  |          iq-type  func unit       |       |       |  |     |  |  |  is_fence        |   |  |
           //     |  |  |  |          |        |               |       |       |  |     |  |  |  |  is_fencei    |   |  |  is breakpoint or ecall?
           //     |  |  |  |          |        |       dst     |       |       |  |     |  |  |  |  |  mem       |   |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |          |        |       regtype |       |       |  |     |  |  |  |  |  cmd       |   |  |  |  |  flush on commit
           //     |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  csr cmd
  // A-type       |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  |
  AMOADD_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N), // TODO make AMOs higherperformance
  AMOXOR_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N),
  AMOSWAP_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N),
  AMOAND_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N),
  AMOOR_W -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N),
  AMOMIN_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMINU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N),
  AMOMAX_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMAXU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N),

  AMOADD_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N),
  AMOXOR_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N),
  AMOSWAP_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N),
  AMOAND_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N),
  AMOOR_D -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N),
  AMOMIN_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMINU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N),
  AMOMAX_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMAXU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N),

  LR_W    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N),
  LR_D    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N),
  SC_W    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N),
  SC_D    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N)
  )
}

/**
 * FP Decode constants
 */
object FDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //    is val inst?                                  rs1 regtype     |  |     |  uses_stq           |    |  |
            //    |  is fp inst?                                |       rs2 type|  |     |  |  is_amo          |    |  |
            //    |  |  is dst single-prec?                     |       |       |  |     |  |  |  is_fence     |    |  |
            //    |  |  |  micro-opcode                         |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //    |  |  |  |           iq_type  func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //    |  |  |  |           |        unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //    |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FLW     -> List(Y, Y, Y, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N),
  FLD     -> List(Y, Y, N, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N),
  FSW     -> List(Y, Y, Y, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N), // sort of a lie; broken into two micro-ops
  FSD     -> List(Y, Y, N, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  FCLASS_S-> List(Y, Y, Y, uopFCLASS_S,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCLASS_D-> List(Y, Y, N, uopFCLASS_D,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMV_W_X -> List(Y, Y, Y, uopFMV_W_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_D_X -> List(Y, Y, N, uopFMV_D_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_X_W -> List(Y, Y, Y, uopFMV_X_W, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_X_D -> List(Y, Y, N, uopFMV_X_D, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FSGNJ_S -> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJ_D -> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJX_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJX_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJN_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJN_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // FP to FP
  FCVT_S_D-> List(Y, Y, Y, uopFCVT_S_D,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_S-> List(Y, Y, N, uopFCVT_D_S,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // Int to FP
  FCVT_S_W-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_WU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_L-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_LU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FCVT_D_W-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_WU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_L-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_LU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // FP to Int
  FCVT_W_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_WU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_L_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_LU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FCVT_W_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_WU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_L_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_LU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // "fp_single" is used for wb_data formatting (and debugging)
  FEQ_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLT_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLE_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FEQ_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLT_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLE_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMIN_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMAX_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMIN_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMAX_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FADD_S   ->List(Y, Y, Y, uopFADD_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSUB_S   ->List(Y, Y, Y, uopFSUB_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMUL_S   ->List(Y, Y, Y, uopFMUL_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FADD_D   ->List(Y, Y, N, uopFADD_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSUB_D   ->List(Y, Y, N, uopFSUB_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMUL_D   ->List(Y, Y, N, uopFMUL_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMADD_S  ->List(Y, Y, Y, uopFMADD_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMSUB_S  ->List(Y, Y, Y, uopFMSUB_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMADD_S ->List(Y, Y, Y, uopFNMADD_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMSUB_S ->List(Y, Y, Y, uopFNMSUB_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMADD_D  ->List(Y, Y, N, uopFMADD_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMSUB_D  ->List(Y, Y, N, uopFMSUB_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMADD_D ->List(Y, Y, N, uopFNMADD_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMSUB_D ->List(Y, Y, N, uopFNMSUB_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}

/**
 * FP Divide SquareRoot Constants
 */
object FDivSqrtDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //     is val inst?                                 rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  is fp inst?                               |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  is dst single-prec?                    |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  micro-opcode                        |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //     |  |  |  |           iq-type func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |           |       unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |           |       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FDIV_S    ->List(Y, Y, Y, uopFDIV_S , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FDIV_D    ->List(Y, Y, N, uopFDIV_D , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSQRT_S   ->List(Y, Y, Y, uopFSQRT_S, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSQRT_D   ->List(Y, Y, N, uopFSQRT_D, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}
//scalastyle:on

/**
 * RoCC initial decode
 */
object RoCCDecode extends DecodeConstants
{
  // Note: We use FU_CSR since CSR instructions cannot co-execute with RoCC instructions
                       //                                                                   frs3_en                        wakeup_delay
                       //     is val inst?                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
                       //     |  is fp inst?                                                |  |     uses_ldq              |    |  is_br
                       //     |  |  is single-prec                          rs1 regtype     |  |     |  uses_stq           |    |  |
                       //     |  |  |                                       |       rs2 type|  |     |  |  is_amo          |    |  |
                       //     |  |  |  micro-code           func unit       |       |       |  |     |  |  |  is_fence     |    |  |
                       //     |  |  |  |           iq-type  |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
                       //     |  |  |  |           |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
                       //     |  |  |  |           |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  val table: Array[(BitPat, List[BitPat])] = Array(//       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
    CUSTOM0            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}


// ------------------------------------------------------------//
// 以上主要定义不同指令的译码结果，即译码表
// 以下开始定义译码模块，即 DecodeUnit
// （后者在core.scala中被实例化）
// ------------------------------------------------------------//

/**
 * IO bundle for the Decode unit
 */
class DecodeUnitIo(implicit p: Parameters) extends BoomBundle
{ //定义DecodeUnit的IO信号
  // 输入、输出分别是译码前后的 MicroOp
  val enq = new Bundle { val uop = Input(new MicroOp()) }
  val deq = new Bundle { val uop = Output(new MicroOp()) }

  // from CSRFile
  val status = Input(new freechips.rocketchip.rocket.MStatus())
  // 读取当前CSR中MStatus的值
  val csr_decode = Flipped(new freechips.rocketchip.rocket.CSRDecodeIO)
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))
}

/**
 * Decode unit that takes in a single instruction and generates a MicroOp.
 */
class DecodeUnit(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{ //定义译码模块，根据单条指令得到译码后的 MicroOp
  // 注意：每条指令在译码单元译码时是相互独立的，在重命名时处理数据依赖关系，
  //      在下面的BranchMaskGenerationLogic中处理控制依赖关系
  val io = IO(new DecodeUnitIo)

  val uop = Wire(new MicroOp())
  uop := io.enq.uop // uop即deq，先用enq赋上初值

  var decode_table = XDecode.table
  if (usingFPU) decode_table ++= FDecode.table
  if (usingFPU && usingFDivSqrt) decode_table ++= FDivSqrtDecode.table
  if (usingRoCC) decode_table ++= RoCCDecode.table
  decode_table ++= (if (xLen == 64) X64Decode.table else X32Decode.table)
  // 根据 BOOM的配置得到译码表

  val inst = uop.inst

  val cs = Wire(new CtrlSigs()).decode(inst, decode_table)
  // 进行译码，单条指令译码的控制信号结果

  // Exception Handling
  io.csr_decode.inst := inst
  val csr_en = cs.csr_cmd.isOneOf(CSR.S, CSR.C, CSR.W)
  val csr_ren = cs.csr_cmd.isOneOf(CSR.S, CSR.C) && uop.lrs1 === 0.U
  val system_insn = cs.csr_cmd === CSR.I
  val sfence = cs.uopc === uopSFENCE

  val cs_legal = cs.legal
//   dontTouch(cs_legal)

  val id_illegal_insn = !cs_legal ||
    cs.fp_val && io.csr_decode.fp_illegal || // TODO check for illegal rm mode: (io.fpu.illegal_rm)
    cs.rocc && io.csr_decode.rocc_illegal ||
    cs.is_amo && !io.status.isa('a'-'a')  ||
    (cs.fp_val && !cs.fp_single) && !io.status.isa('d'-'a') ||
    csr_en && (io.csr_decode.read_illegal || !csr_ren && io.csr_decode.write_illegal) ||
    ((sfence || system_insn) && io.csr_decode.system_illegal)
  // 分情况讨论，判断是否为“非法指令”，若是则触发异常

//     cs.div && !csr.io.status.isa('m'-'a') || TODO check for illegal div instructions

  def checkExceptions(x: Seq[(Bool, UInt)]) = {
    (x.map(_._1).reduce(_||_), PriorityMux(x))
    // 返回一个(Bool: xcpt_valid, UInt: xcpt_cause)的元组
    // 前者：提取出Seq中所有Bool进行一个缩减或，表示当前有无异常发生（任何异常）
    // 后者：对x使用优先级多选器，选出优先级最高的那个UInt值（优先级最高的异常号）
  }

  val (xcpt_valid, xcpt_cause) = checkExceptions(List(
    (io.interrupt && !io.enq.uop.is_sfb, io.interrupt_cause),  // Disallow interrupts while we are handling a SFB
    (uop.bp_debug_if,                    (CSR.debugTriggerCause).U),
    (uop.bp_xcpt_if,                     (Causes.breakpoint).U),
    (uop.xcpt_pf_if,                     (Causes.fetch_page_fault).U),
    (uop.xcpt_ae_if,                     (Causes.fetch_access).U),
    (id_illegal_insn,                    (Causes.illegal_instruction).U)))
  // 优先级：SFB > bp_debug > bp_xcpt > 取指page falult > 取指地址异常 > 非法指令

  uop.exception := xcpt_valid
  uop.exc_cause := xcpt_cause

  //-------------------------------------------------------------
  // 下面主要对uop（即deq）进行赋值，根据控制信号和指令本身
  uop.uopc       := cs.uopc
  uop.iq_type    := cs.iq_type
  uop.fu_code    := cs.fu_code

  // x-registers placed in 0-31, f-registers placed in 32-63.
  // This allows us to straight-up compare register specifiers and not need to
  // verify the rtypes (e.g., bypassing in rename).
  uop.ldst       := inst(RD_MSB,RD_LSB)    // 第07-11位
  uop.lrs1       := inst(RS1_MSB,RS1_LSB)  // 第15-19位
  uop.lrs2       := inst(RS2_MSB,RS2_LSB)  // 第20-24位
  uop.lrs3       := inst(RS3_MSB,RS3_LSB)  // 第27-31位

  uop.ldst_val   := cs.dst_type =/= RT_X && !(uop.ldst === 0.U && uop.dst_rtype === RT_FIX)
  // 排除情况：定点指令写回0号寄存器
  uop.dst_rtype  := cs.dst_type
  uop.lrs1_rtype := cs.rs1_type
  uop.lrs2_rtype := cs.rs2_type
  uop.frs3_en    := cs.frs3_en

  // 以下与SFB优化相关，暂未研究（TODO）
  uop.ldst_is_rs1 := uop.is_sfb_shadow
  // SFB optimization
  when (uop.is_sfb_shadow && cs.rs2_type === RT_X) {
    uop.lrs2_rtype  := RT_FIX
    uop.lrs2        := inst(RD_MSB,RD_LSB)
    uop.ldst_is_rs1 := false.B
  } .elsewhen (uop.is_sfb_shadow && cs.uopc === uopADD && inst(RS1_MSB,RS1_LSB) === 0.U) {
    uop.uopc        := uopMOV
    uop.lrs1        := inst(RD_MSB, RD_LSB)
    uop.ldst_is_rs1 := true.B
  }
  when (uop.is_sfb_br) {
    uop.fu_code := FU_JMP
  }


  uop.fp_val     := cs.fp_val
  uop.fp_single  := cs.fp_single // TODO use this signal instead of the FPU decode's table signal?

  uop.mem_cmd    := cs.mem_cmd
  uop.mem_size   := Mux(cs.mem_cmd.isOneOf(M_SFENCE, M_FLUSH_ALL), Cat(uop.lrs2 =/= 0.U, uop.lrs1 =/= 0.U), inst(13,12))
  uop.mem_signed := !inst(14)
  uop.uses_ldq   := cs.uses_ldq
  uop.uses_stq   := cs.uses_stq
  uop.is_amo     := cs.is_amo
  uop.is_fence   := cs.is_fence
  uop.is_fencei  := cs.is_fencei
  uop.is_sys_pc2epc   := cs.is_sys_pc2epc
  uop.is_unique  := cs.inst_unique
  uop.flush_on_commit := cs.flush_on_commit || (csr_en && !csr_ren && io.csr_decode.write_flush)

  uop.bypassable   := cs.bypassable

  //-------------------------------------------------------------
  // immediates

  // repackage the immediate, and then pass the fewest number of bits around
  val di24_20 = Mux(cs.imm_sel === IS_B || cs.imm_sel === IS_S, inst(11,7), inst(24,20))
  // 两种格式：SB类型（branch）、S类型（store）
  uop.imm_packed := Cat(inst(31,25), di24_20, inst(19,12))

  //-------------------------------------------------------------

  uop.is_br          := cs.is_br
  uop.is_jal         := (uop.uopc === uopJAL)
  uop.is_jalr        := (uop.uopc === uopJALR)
  // uop.is_jump        := cs.is_jal || (uop.uopc === uopJALR)
  // uop.is_ret         := (uop.uopc === uopJALR) &&
  //                       (uop.ldst === X0) &&
  //                       (uop.lrs1 === RA)
  // uop.is_call        := (uop.uopc === uopJALR || uop.uopc === uopJAL) &&
  //                       (uop.ldst === RA)

  //-------------------------------------------------------------

  io.deq.uop := uop
}

// ------------------------------------------------------------//
// 以下开始定义分支译码模块，即 BranchDecode
// （在frontend.scala中被实例化）
// ------------------------------------------------------------//

/**
 * Smaller Decode unit for the Frontend to decode different
 * branches.
 * Accepts EXPANDED RVC instructions
  */

class BranchDecodeSignals(implicit p: Parameters) extends BoomBundle
{ //定义BranchDecode的Output信号
  val is_ret   = Bool()
  val is_call  = Bool()
  val target   = UInt(vaddrBitsExtended.W)
  val cfi_type = UInt(CFI_SZ.W)


  // Is this branch a short forwards jump?
  val sfb_offset = Valid(UInt(log2Ceil(icBlockBytes).W))
  // Is this instruction allowed to be inside a sfb?
  val shadowable = Bool()
}

class BranchDecode(implicit p: Parameters) extends BoomModule
{ //定义分支译码模块，输入当前指令和PC
  val io = IO(new Bundle {
    val inst    = Input(UInt(32.W))
    val pc      = Input(UInt(vaddrBitsExtended.W))

    val out = Output(new BranchDecodeSignals)
  })

  val bpd_csignals = //同样调用DecodeLogic译码得到控制信号
    freechips.rocketchip.rocket.DecodeLogic(io.inst,    //当前指令
                  List[BitPat](N, N, N, N, X),          //默认值
////                               is br?
////                               |  is jal?
////                               |  |  is jalr?
////                               |  |  |
////                               |  |  |  shadowable
////                               |  |  |  |  has_rs2
////                               |  |  |  |  |
            Array[(BitPat, List[BitPat])](              //译码表
               JAL         -> List(N, Y, N, N, X),
               JALR        -> List(N, N, Y, N, X),
               BEQ         -> List(Y, N, N, N, X),
               BNE         -> List(Y, N, N, N, X),
               BGE         -> List(Y, N, N, N, X),
               BGEU        -> List(Y, N, N, N, X),
               BLT         -> List(Y, N, N, N, X),
               BLTU        -> List(Y, N, N, N, X),

               SLLI        -> List(N, N, N, Y, N),
               SRLI        -> List(N, N, N, Y, N),
               SRAI        -> List(N, N, N, Y, N),

               ADDIW       -> List(N, N, N, Y, N),
               SLLIW       -> List(N, N, N, Y, N),
               SRAIW       -> List(N, N, N, Y, N),
               SRLIW       -> List(N, N, N, Y, N),

               ADDW        -> List(N, N, N, Y, Y),
               SUBW        -> List(N, N, N, Y, Y),
               SLLW        -> List(N, N, N, Y, Y),
               SRAW        -> List(N, N, N, Y, Y),
               SRLW        -> List(N, N, N, Y, Y),

               LUI         -> List(N, N, N, Y, N),

               ADDI        -> List(N, N, N, Y, N),
               ANDI        -> List(N, N, N, Y, N),
               ORI         -> List(N, N, N, Y, N),
               XORI        -> List(N, N, N, Y, N),
               SLTI        -> List(N, N, N, Y, N),
               SLTIU       -> List(N, N, N, Y, N),

               SLL         -> List(N, N, N, Y, Y),
               ADD         -> List(N, N, N, Y, Y),
               SUB         -> List(N, N, N, Y, Y),
               SLT         -> List(N, N, N, Y, Y),
               SLTU        -> List(N, N, N, Y, Y),
               AND         -> List(N, N, N, Y, Y),
               OR          -> List(N, N, N, Y, Y),
               XOR         -> List(N, N, N, Y, Y),
               SRA         -> List(N, N, N, Y, Y),
               SRL         -> List(N, N, N, Y, Y)
            ))

  val cs_is_br = bpd_csignals(0)(0)
  val cs_is_jal = bpd_csignals(1)(0)
  val cs_is_jalr = bpd_csignals(2)(0)
  val cs_is_shadowable = bpd_csignals(3)(0)
  val cs_has_rs2 = bpd_csignals(4)(0)

  // 下面主要根据译码结果对输出进行赋值
  io.out.is_call := (cs_is_jal || cs_is_jalr) && GetRd(io.inst) === RA
  // call指令：jal 或 jalr 且 目标寄存器为1号
  io.out.is_ret  := cs_is_jalr && GetRs1(io.inst) === BitPat("b00?01") && GetRd(io.inst) === X0
  // ret指令：jalr 且 读00x01 且 写0号寄存器

  io.out.target := Mux(cs_is_br, ComputeBranchTarget(io.pc, io.inst, xLen),
                                 ComputeJALTarget(io.pc, io.inst, xLen))
  // 两种格式的地址：branch、jump

  io.out.cfi_type :=
    Mux(cs_is_jalr,
      CFI_JALR,
    Mux(cs_is_jal,
      CFI_JAL,
    Mux(cs_is_br,
      CFI_BR,
      CFI_X)))

  // 以下与SFB优化相关，暂未研究（TODO）
  val br_offset = Cat(io.inst(7), io.inst(30,25), io.inst(11,8), 0.U(1.W))
  // Is a sfb if it points forwards (offset is positive)
  io.out.sfb_offset.valid := cs_is_br && !io.inst(31) && br_offset =/= 0.U && (br_offset >> log2Ceil(icBlockBytes)) === 0.U
  io.out.sfb_offset.bits  := br_offset
  io.out.shadowable := cs_is_shadowable && (
    !cs_has_rs2 ||
    (GetRs1(io.inst) === GetRd(io.inst)) ||
    (io.inst === ADD && GetRs1(io.inst) === X0)
  )
}

// ------------------------------------------------------------//
// 以下开始定义分支掩码生成模块，即 BranchMaskGenerationLogic
// （在core.scala中被实例化）
// ------------------------------------------------------------//

/**
 * Track the current "branch mask", and give out the branch mask to each micro-op in Decode
 * (each micro-op in the machine has a branch mask which says which branches it
 * is being speculated under).
 *
 * @param pl_width pipeline width for the processor
 */
class BranchMaskGenerationLogic(val pl_width: Int)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // guess if the uop is a branch (we'll catch this later)
    val is_branch = Input(Vec(pl_width, Bool())) // 该pipeline slot是否为分支
    // lock in that it's actually a branch and will fire, so we update
    // the branch_masks.
    val will_fire = Input(Vec(pl_width, Bool())) // 这条指令是否会发射（还是阻塞）

    // give out tag immediately (needed in rename)
    // mask can come later in the cycle
    val br_tag    = Output(Vec(pl_width, UInt(brTagSz.W)))    // 这条指令的“分支标签”
    val br_mask   = Output(Vec(pl_width, UInt(maxBrCount.W))) // 后续指令的“分支掩码”，独热码

    // tell decoders the branch mask has filled up, but on the granularity
    // of an individual micro-op (so some micro-ops can go through)
    val is_full   = Output(Vec(pl_width, Bool())) // 分支掩码使用完了

    val brupdate         = Input(new BrUpdateInfo()) // 分支解决信息
    val flush_pipeline = Input(Bool())

    val debug_branch_mask = Output(UInt(maxBrCount.W))
  })

  val branch_mask = RegInit(0.U(maxBrCount.W)) // 流水线寄存器，初始化为0
  // 表示当前指令能看到的branch，若某位为1，表示受对应这条分支指令的影响

  //-------------------------------------------------------------
  // Give out the branch tag to each branch micro-op

  var allocate_mask = branch_mask // 分配的中间结果
  val tag_masks = Wire(Vec(pl_width, UInt(maxBrCount.W)))

  for (w <- 0 until pl_width) {
    // TODO this is a loss of performance as we're blocking branches based on potentially fake branches
    io.is_full(w) := (allocate_mask === ~(0.U(maxBrCount.W))) && io.is_branch(w)
    // 已满：allocate_mask全1 且 当前是条分支指令

    // find br_tag and compute next br_mask
    val new_br_tag = Wire(UInt(brTagSz.W))
    new_br_tag := 0.U
    tag_masks(w) := 0.U

    for (i <- maxBrCount-1 to 0 by -1) {
      when (~allocate_mask(i)) { // 从大到小遍历，看是否为0（未分配）
        new_br_tag := i.U
        tag_masks(w) := (1.U << i.U) // 分配的mask，后面会进行一个或
      }
    }

    io.br_tag(w) := new_br_tag // 输出新分配的分支标签
    allocate_mask = Mux(io.is_branch(w), tag_masks(w) | allocate_mask, allocate_mask)
    // 更新中间结果
  }

  //-------------------------------------------------------------
  // Give out the branch mask to each micro-op
  // (kill off the bits that corresponded to branches that aren't going to fire)

  var curr_mask = branch_mask
  for (w <- 0 until pl_width) {
    io.br_mask(w) := GetNewBrMask(io.brupdate, curr_mask)
    // 消除该已解决的分支的影响：（用于输出br_mask，可以理解为下面GetNewBrMask的bypass）
    // curr_mask & ~brupdate.b1.resolve_mask
    curr_mask = Mux(io.will_fire(w), tag_masks(w) | curr_mask, curr_mask)
    // 这周期分配出去的分支掩码，进行一个或
  }

  //-------------------------------------------------------------
  // Update the current branch_mask

  when (io.flush_pipeline) {
    branch_mask := 0.U // 清空流水线同时也清空分支掩码，当前看不到分支指令
  } .otherwise {
    val mask = Mux(io.brupdate.b2.mispredict,
      io.brupdate.b2.uop.br_mask, // 发生错误的推测，该分支后的所有分支失效，分支掩码回退至该br_mask
      ~(0.U(maxBrCount.W))) // 正确推测，不变（与上全1）
    branch_mask := GetNewBrMask(io.brupdate, curr_mask) & mask
    // 消除该已解决的分支的影响：（用于更新branch_mask）
    // curr_mask & ~brupdate.b1.resolve_mask
  }

  io.debug_branch_mask := branch_mask
}
