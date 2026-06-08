/* Auto-generated from Infineon DAVE device pack TC1797_v1_0.dip
 * (TC1797.REGS, REV 20070702, TS Rev. 1.4). Faithful power-on reset values
 * for TC1797 SFRs; don't-care (revision) nibbles masked to 0. A small set of
 * transient hardware-status bits (CCUCONx.LCK) are masked to their settled,
 * firmware-visible value -- a config read-back shadow cannot clear a HW-driven
 * status bit, so seeding it set would hang a read-modify-write+poll. See the
 * generator's SEED_STATUS_MASK. Do NOT edit by hand -- regenerate from the pack. */
#ifndef HW_TRICORE_TC1797_RESET_SEED_H
#define HW_TRICORE_TC1797_RESET_SEED_H
#include <stdint.h>
typedef struct { uint32_t addr; uint32_t val; } TC1797ResetSeed;
static const TC1797ResetSeed tc1797_reset_seed[] = {
    /* ---- Buses ---- */
    { 0xF0000108u, 0x00006A00u },  /* SBCU_ID (rev nibble=0) */
    { 0xF0000110u, 0x4009FFFFu },  /* SBCU_CON */
    { 0xF0000130u, 0x00007003u },  /* SBCU_DBCNTL */
    { 0xF0000134u, 0x0000FFFFu },  /* SBCU_DBGRNT */
    { 0xF0000144u, 0x0000FFFFu },  /* SBCU_DBGNTT */
    { 0xF000014Cu, 0x00003180u },  /* SBCU_DBBOST */
    /* ---- STM ---- */
    { 0xF0000200u, 0x00000200u },  /* STM_CLC */
    { 0xF0000208u, 0x0000C000u },  /* STM_ID (rev nibble=0) */
    /* ---- Cerberus ---- */
    { 0xF0000464u, 0x1015A083u },  /* CBS_JTAGID */
    { 0xF0000470u, 0x00600000u },  /* CBS_MCDBBS */
    { 0xF0000480u, 0x00010000u },  /* CBS_OSTATE */
    { 0xF000048Cu, 0x0010F068u },  /* CBS_ICTTA */
    /* ---- SCU ---- */
    { 0xF0000508u, 0x0052C000u },  /* SCU_ID (rev nibble=0) */
    { 0xF0000510u, 0x0000001Cu },  /* SCU_OSCCON */
    { 0xF0000514u, 0x00000009u },  /* SCU_PLLSTAT */
    { 0xF0000518u, 0x0001C600u },  /* SCU_PLLCON0 */
    { 0xF000051Cu, 0x0002000Fu },  /* SCU_PLLCON1 */
    { 0xF0000520u, 0x0000B80Fu },  /* SCU_PLLERAYCTR */
    { 0xF0000530u, 0x00000001u },  /* SCU_CCUCON0 (LCK status bit masked) */
    { 0xF0000534u, 0x00000B01u },  /* SCU_CCUCON1 (LCK status bit masked) */
    { 0xF0000550u, 0x00010000u },  /* SCU_RSTSTAT */
    { 0xF0000554u, 0x05BE05BEu },  /* SCU_RSTCNTCON */
    { 0xF0000558u, 0x000002A2u },  /* SCU_RSTCON */
    { 0xF0000570u, 0x00000100u },  /* SCU_ESRCFG0 */
    { 0xF0000574u, 0x00000080u },  /* SCU_ESRCFG1 */
    { 0xF0000578u, 0x00000100u },  /* SCU_ESRCFG2 */
    { 0xF0000590u, 0x0000000Fu },  /* SCU_PDRR */
    { 0xF00005A0u, 0x002010E0u },  /* SCU_IOCR */
    { 0xF00005B0u, 0x00000100u },  /* SCU_PMCSR */
    { 0xF00005C0u, 0x00008000u },  /* SCU_STSTAT */
    { 0xF00005F0u, 0xFFFC0002u },  /* WDT_CON0 */
    { 0xF00005F8u, 0xFFFC0010u },  /* WDT_SR */
    { 0xF0000630u, 0x0000FFFFu },  /* SCU_TRAPDIS */
    { 0xF0000640u, 0x00009101u },  /* SCU_CHIPID */
    { 0xF0000644u, 0x00001820u },  /* SCU_MANID */
    /* ---- MSC ---- */
    { 0xF0000800u, 0x00000003u },  /* MSC0_CLC */
    { 0xF0000808u, 0x0028C000u },  /* MSC0_ID (rev nibble=0) */
    { 0xF0000900u, 0x00000003u },  /* MSC1_CLC */
    { 0xF0000908u, 0x0028C000u },  /* MSC1_ID (rev nibble=0) */
    /* ---- ASC ---- */
    { 0xF0000A00u, 0x00000003u },  /* ASC0_CLC */
    { 0xF0000A08u, 0x00004400u },  /* ASC0_ID (rev nibble=0) */
    { 0xF0000B08u, 0x00004400u },  /* ASC1_ID (rev nibble=0) */
    /* ---- Ports ---- */
    { 0xF0000C10u, 0x20202020u },  /* P0_IOCR0 */
    { 0xF0000C14u, 0x20202020u },  /* P0_IOCR4 */
    { 0xF0000C18u, 0x20202020u },  /* P0_IOCR8 */
    { 0xF0000C1Cu, 0x20202020u },  /* P0_IOCR12 */
    { 0xF0000D10u, 0x20202020u },  /* P1_IOCR0 */
    { 0xF0000D14u, 0x20202020u },  /* P1_IOCR4 */
    { 0xF0000D18u, 0x20202020u },  /* P1_IOCR8 */
    { 0xF0000D1Cu, 0x20202020u },  /* P1_IOCR12 */
    { 0xF0000E10u, 0x20202020u },  /* P2_IOCR0 */
    { 0xF0000E14u, 0x20202020u },  /* P2_IOCR4 */
    { 0xF0000E18u, 0x20202020u },  /* P2_IOCR8 */
    { 0xF0000E1Cu, 0x20202020u },  /* P2_IOCR12 */
    { 0xF0000F10u, 0x20202020u },  /* P3_IOCR0 */
    { 0xF0000F14u, 0x20202020u },  /* P3_IOCR4 */
    { 0xF0000F18u, 0x20202020u },  /* P3_IOCR8 */
    { 0xF0000F1Cu, 0x20202020u },  /* P3_IOCR12 */
    { 0xF0001010u, 0x20202020u },  /* P4_IOCR0 */
    { 0xF0001014u, 0x20202020u },  /* P4_IOCR4 */
    { 0xF0001018u, 0x20202020u },  /* P4_IOCR8 */
    { 0xF000101Cu, 0x20202020u },  /* P4_IOCR12 */
    { 0xF0001110u, 0x20202020u },  /* P5_IOCR0 */
    { 0xF0001114u, 0x20202020u },  /* P5_IOCR4 */
    { 0xF0001118u, 0x20202020u },  /* P5_IOCR8 */
    { 0xF000111Cu, 0x20202020u },  /* P5_IOCR12 */
    { 0xF0001210u, 0x20202020u },  /* P6_IOCR0 */
    { 0xF0001214u, 0x20202020u },  /* P6_IOCR4 */
    { 0xF0001218u, 0x20202020u },  /* P6_IOCR8 */
    { 0xF000121Cu, 0x20202020u },  /* P6_IOCR12 */
    { 0xF0001310u, 0x20202020u },  /* P7_IOCR0 */
    { 0xF0001314u, 0x20202020u },  /* P7_IOCR4 */
    { 0xF0001410u, 0x20202020u },  /* P8_IOCR0 */
    { 0xF0001414u, 0x20202020u },  /* P8_IOCR4 */
    { 0xF0001510u, 0x20202020u },  /* P9_IOCR0 */
    { 0xF0001514u, 0x20202020u },  /* P9_IOCR4 */
    { 0xF0001518u, 0x20202020u },  /* P9_IOCR8 */
    { 0xF000151Cu, 0x20202020u },  /* P9_IOCR12 */
    { 0xF0001610u, 0x20202020u },  /* P10_IOCR0 */
    { 0xF0001614u, 0x20202020u },  /* P10_IOCR4 */
    { 0xF0001710u, 0x20202020u },  /* P11_IOCR0 */
    { 0xF0001714u, 0x20202020u },  /* P11_IOCR4 */
    { 0xF0001718u, 0x20202020u },  /* P11_IOCR8 */
    { 0xF000171Cu, 0x20202020u },  /* P11_IOCR12 */
    /* ---- GPTA ---- */
    { 0xF0001800u, 0x00000003u },  /* GPTA0_CLC */
    { 0xF0001808u, 0x0029C000u },  /* GPTA0_ID (rev nibble=0) */
    { 0xF00018D8u, 0x0000FFFFu },  /* GPTA0_CKBCTR */
    { 0xF0002008u, 0x0029C000u },  /* GPTA1_ID (rev nibble=0) */
    { 0xF00020D8u, 0x0000FFFFu },  /* GPTA1_CKBCTR */
    { 0xF0002808u, 0x002AC000u },  /* LTCA2_ID (rev nibble=0) */
    /* ---- DMA ---- */
    { 0xF0003C00u, 0x00000008u },  /* DMA_CLC */
    { 0xF0003C08u, 0x001AC000u },  /* DMA_ID (rev nibble=0) */
    /* ---- MultiCAN ---- */
    { 0xF0004000u, 0x00000003u },  /* CAN_CLC */
    { 0xF0004100u, 0x007F7F00u },  /* CAN_LIST0 */
    { 0xF0004104u, 0x01000000u },  /* CAN_LIST1 */
    { 0xF0004108u, 0x01000000u },  /* CAN_LIST2 */
    { 0xF000410Cu, 0x01000000u },  /* CAN_LIST3 */
    { 0xF0004110u, 0x01000000u },  /* CAN_LIST4 */
    { 0xF0004114u, 0x01000000u },  /* CAN_LIST5 */
    { 0xF0004118u, 0x01000000u },  /* CAN_LIST6 */
    { 0xF000411Cu, 0x01000000u },  /* CAN_LIST7 */
    { 0xF0004180u, 0x00000020u },  /* CAN_MSID0 */
    { 0xF0004184u, 0x00000020u },  /* CAN_MSID1 */
    { 0xF0004188u, 0x00000020u },  /* CAN_MSID2 */
    { 0xF000418Cu, 0x00000020u },  /* CAN_MSID3 */
    { 0xF0004190u, 0x00000020u },  /* CAN_MSID4 */
    { 0xF0004194u, 0x00000020u },  /* CAN_MSID5 */
    { 0xF0004198u, 0x00000020u },  /* CAN_MSID6 */
    { 0xF000419Cu, 0x00000020u },  /* CAN_MSID7 */
    { 0xF00041C4u, 0x00000301u },  /* CAN_PANCTR */
    { 0xF0004200u, 0x00000001u },  /* CAN_NCR0 */
    { 0xF0004214u, 0x00600000u },  /* CAN_NECNT0 */
    { 0xF0004300u, 0x00000001u },  /* CAN_NCR1 */
    { 0xF0004314u, 0x00600000u },  /* CAN_NECNT1 */
    { 0xF0004400u, 0x00000001u },  /* CAN_NCR2 */
    { 0xF0004414u, 0x00600000u },  /* CAN_NECNT2 */
    { 0xF0004500u, 0x00000001u },  /* CAN_NCR3 */
    { 0xF0004514u, 0x00600000u },  /* CAN_NECNT3 */
    { 0xF000500Cu, 0x3FFFFFFFu },  /* CAN_MOAMR0 */
    { 0xF000501Cu, 0x01000000u },  /* CAN_MOCTR0 */
    { 0xF000502Cu, 0x3FFFFFFFu },  /* CAN_MOAMR1 */
    { 0xF000503Cu, 0x02000000u },  /* CAN_MOCTR1 */
    { 0xF000504Cu, 0x3FFFFFFFu },  /* CAN_MOAMR2 */
    { 0xF000505Cu, 0x03010000u },  /* CAN_MOCTR2 */
    { 0xF000506Cu, 0x3FFFFFFFu },  /* CAN_MOAMR3 */
    { 0xF000507Cu, 0x04020000u },  /* CAN_MOCTR3 */
    { 0xF000508Cu, 0x3FFFFFFFu },  /* CAN_MOAMR4 */
    { 0xF000509Cu, 0x05030000u },  /* CAN_MOCTR4 */
    { 0xF00050ACu, 0x3FFFFFFFu },  /* CAN_MOAMR5 */
    { 0xF00050BCu, 0x06040000u },  /* CAN_MOCTR5 */
    { 0xF00050CCu, 0x3FFFFFFFu },  /* CAN_MOAMR6 */
    { 0xF00050DCu, 0x07050000u },  /* CAN_MOCTR6 */
    { 0xF00050ECu, 0x3FFFFFFFu },  /* CAN_MOAMR7 */
    { 0xF00050FCu, 0x08060000u },  /* CAN_MOCTR7 */
    { 0xF000510Cu, 0x3FFFFFFFu },  /* CAN_MOAMR8 */
    { 0xF000511Cu, 0x09070000u },  /* CAN_MOCTR8 */
    { 0xF000512Cu, 0x3FFFFFFFu },  /* CAN_MOAMR9 */
    { 0xF000513Cu, 0x0A080000u },  /* CAN_MOCTR9 */
    { 0xF000514Cu, 0x3FFFFFFFu },  /* CAN_MOAMR10 */
    { 0xF000515Cu, 0x0B090000u },  /* CAN_MOCTR10 */
    { 0xF000516Cu, 0x3FFFFFFFu },  /* CAN_MOAMR11 */
    { 0xF000517Cu, 0x0C0A0000u },  /* CAN_MOCTR11 */
    { 0xF000518Cu, 0x3FFFFFFFu },  /* CAN_MOAMR12 */
    { 0xF000519Cu, 0x0D0B0000u },  /* CAN_MOCTR12 */
    { 0xF00051ACu, 0x3FFFFFFFu },  /* CAN_MOAMR13 */
    { 0xF00051BCu, 0x0E0C0000u },  /* CAN_MOCTR13 */
    { 0xF00051CCu, 0x3FFFFFFFu },  /* CAN_MOAMR14 */
    { 0xF00051DCu, 0x0F0D0000u },  /* CAN_MOCTR14 */
    { 0xF00051ECu, 0x3FFFFFFFu },  /* CAN_MOAMR15 */
    { 0xF00051FCu, 0x100E0000u },  /* CAN_MOCTR15 */
    { 0xF000520Cu, 0x3FFFFFFFu },  /* CAN_MOAMR16 */
    { 0xF000521Cu, 0x110F0000u },  /* CAN_MOCTR16 */
    { 0xF000522Cu, 0x3FFFFFFFu },  /* CAN_MOAMR17 */
    { 0xF000523Cu, 0x12100000u },  /* CAN_MOCTR17 */
    { 0xF000524Cu, 0x3FFFFFFFu },  /* CAN_MOAMR18 */
    { 0xF000525Cu, 0x13110000u },  /* CAN_MOCTR18 */
    { 0xF000526Cu, 0x3FFFFFFFu },  /* CAN_MOAMR19 */
    { 0xF000527Cu, 0x14120000u },  /* CAN_MOCTR19 */
    { 0xF000528Cu, 0x3FFFFFFFu },  /* CAN_MOAMR20 */
    { 0xF000529Cu, 0x15130000u },  /* CAN_MOCTR20 */
    { 0xF00052ACu, 0x3FFFFFFFu },  /* CAN_MOAMR21 */
    { 0xF00052BCu, 0x16140000u },  /* CAN_MOCTR21 */
    { 0xF00052CCu, 0x3FFFFFFFu },  /* CAN_MOAMR22 */
    { 0xF00052DCu, 0x17150000u },  /* CAN_MOCTR22 */
    { 0xF00052ECu, 0x3FFFFFFFu },  /* CAN_MOAMR23 */
    { 0xF00052FCu, 0x18160000u },  /* CAN_MOCTR23 */
    { 0xF000530Cu, 0x3FFFFFFFu },  /* CAN_MOAMR24 */
    { 0xF000531Cu, 0x19170000u },  /* CAN_MOCTR24 */
    { 0xF000532Cu, 0x3FFFFFFFu },  /* CAN_MOAMR25 */
    { 0xF000533Cu, 0x1A180000u },  /* CAN_MOCTR25 */
    { 0xF000534Cu, 0x3FFFFFFFu },  /* CAN_MOAMR26 */
    { 0xF000535Cu, 0x1B190000u },  /* CAN_MOCTR26 */
    { 0xF000536Cu, 0x3FFFFFFFu },  /* CAN_MOAMR27 */
    { 0xF000537Cu, 0x1C1A0000u },  /* CAN_MOCTR27 */
    { 0xF000538Cu, 0x3FFFFFFFu },  /* CAN_MOAMR28 */
    { 0xF000539Cu, 0x1D1B0000u },  /* CAN_MOCTR28 */
    { 0xF00053ACu, 0x3FFFFFFFu },  /* CAN_MOAMR29 */
    { 0xF00053BCu, 0x1E1C0000u },  /* CAN_MOCTR29 */
    { 0xF00053CCu, 0x3FFFFFFFu },  /* CAN_MOAMR30 */
    { 0xF00053DCu, 0x1F1D0000u },  /* CAN_MOCTR30 */
    { 0xF00053ECu, 0x3FFFFFFFu },  /* CAN_MOAMR31 */
    { 0xF00053FCu, 0x201E0000u },  /* CAN_MOCTR31 */
    { 0xF000540Cu, 0x3FFFFFFFu },  /* CAN_MOAMR32 */
    { 0xF000541Cu, 0x211F0000u },  /* CAN_MOCTR32 */
    { 0xF000542Cu, 0x3FFFFFFFu },  /* CAN_MOAMR33 */
    { 0xF000543Cu, 0x22200000u },  /* CAN_MOCTR33 */
    { 0xF000544Cu, 0x3FFFFFFFu },  /* CAN_MOAMR34 */
    { 0xF000545Cu, 0x23210000u },  /* CAN_MOCTR34 */
    { 0xF000546Cu, 0x3FFFFFFFu },  /* CAN_MOAMR35 */
    { 0xF000547Cu, 0x24220000u },  /* CAN_MOCTR35 */
    { 0xF000548Cu, 0x3FFFFFFFu },  /* CAN_MOAMR36 */
    { 0xF000549Cu, 0x25230000u },  /* CAN_MOCTR36 */
    { 0xF00054ACu, 0x3FFFFFFFu },  /* CAN_MOAMR37 */
    { 0xF00054BCu, 0x26240000u },  /* CAN_MOCTR37 */
    { 0xF00054CCu, 0x3FFFFFFFu },  /* CAN_MOAMR38 */
    { 0xF00054DCu, 0x27250000u },  /* CAN_MOCTR38 */
    { 0xF00054ECu, 0x3FFFFFFFu },  /* CAN_MOAMR39 */
    { 0xF00054FCu, 0x28260000u },  /* CAN_MOCTR39 */
    { 0xF000550Cu, 0x3FFFFFFFu },  /* CAN_MOAMR40 */
    { 0xF000551Cu, 0x29270000u },  /* CAN_MOCTR40 */
    { 0xF000552Cu, 0x3FFFFFFFu },  /* CAN_MOAMR41 */
    { 0xF000553Cu, 0x2A280000u },  /* CAN_MOCTR41 */
    { 0xF000554Cu, 0x3FFFFFFFu },  /* CAN_MOAMR42 */
    { 0xF000555Cu, 0x2B290000u },  /* CAN_MOCTR42 */
    { 0xF000556Cu, 0x3FFFFFFFu },  /* CAN_MOAMR43 */
    { 0xF000557Cu, 0x2C2A0000u },  /* CAN_MOCTR43 */
    { 0xF000558Cu, 0x3FFFFFFFu },  /* CAN_MOAMR44 */
    { 0xF000559Cu, 0x2D2B0000u },  /* CAN_MOCTR44 */
    { 0xF00055ACu, 0x3FFFFFFFu },  /* CAN_MOAMR45 */
    { 0xF00055BCu, 0x2E2C0000u },  /* CAN_MOCTR45 */
    { 0xF00055CCu, 0x3FFFFFFFu },  /* CAN_MOAMR46 */
    { 0xF00055DCu, 0x2F2D0000u },  /* CAN_MOCTR46 */
    { 0xF00055ECu, 0x3FFFFFFFu },  /* CAN_MOAMR47 */
    { 0xF00055FCu, 0x302E0000u },  /* CAN_MOCTR47 */
    { 0xF000560Cu, 0x3FFFFFFFu },  /* CAN_MOAMR48 */
    { 0xF000561Cu, 0x312F0000u },  /* CAN_MOCTR48 */
    { 0xF000562Cu, 0x3FFFFFFFu },  /* CAN_MOAMR49 */
    { 0xF000563Cu, 0x32300000u },  /* CAN_MOCTR49 */
    { 0xF000564Cu, 0x3FFFFFFFu },  /* CAN_MOAMR50 */
    { 0xF000565Cu, 0x33310000u },  /* CAN_MOCTR50 */
    { 0xF000566Cu, 0x3FFFFFFFu },  /* CAN_MOAMR51 */
    { 0xF000567Cu, 0x34320000u },  /* CAN_MOCTR51 */
    { 0xF000568Cu, 0x3FFFFFFFu },  /* CAN_MOAMR52 */
    { 0xF000569Cu, 0x35330000u },  /* CAN_MOCTR52 */
    { 0xF00056ACu, 0x3FFFFFFFu },  /* CAN_MOAMR53 */
    { 0xF00056BCu, 0x36340000u },  /* CAN_MOCTR53 */
    { 0xF00056CCu, 0x3FFFFFFFu },  /* CAN_MOAMR54 */
    { 0xF00056DCu, 0x37350000u },  /* CAN_MOCTR54 */
    { 0xF00056ECu, 0x3FFFFFFFu },  /* CAN_MOAMR55 */
    { 0xF00056FCu, 0x38360000u },  /* CAN_MOCTR55 */
    { 0xF000570Cu, 0x3FFFFFFFu },  /* CAN_MOAMR56 */
    { 0xF000571Cu, 0x39370000u },  /* CAN_MOCTR56 */
    { 0xF000572Cu, 0x3FFFFFFFu },  /* CAN_MOAMR57 */
    { 0xF000573Cu, 0x3A380000u },  /* CAN_MOCTR57 */
    { 0xF000574Cu, 0x3FFFFFFFu },  /* CAN_MOAMR58 */
    { 0xF000575Cu, 0x3B390000u },  /* CAN_MOCTR58 */
    { 0xF000576Cu, 0x3FFFFFFFu },  /* CAN_MOAMR59 */
    { 0xF000577Cu, 0x3C3A0000u },  /* CAN_MOCTR59 */
    { 0xF000578Cu, 0x3FFFFFFFu },  /* CAN_MOAMR60 */
    { 0xF000579Cu, 0x3D3B0000u },  /* CAN_MOCTR60 */
    { 0xF00057ACu, 0x3FFFFFFFu },  /* CAN_MOAMR61 */
    { 0xF00057BCu, 0x3E3C0000u },  /* CAN_MOCTR61 */
    { 0xF00057CCu, 0x3FFFFFFFu },  /* CAN_MOAMR62 */
    { 0xF00057DCu, 0x3F3D0000u },  /* CAN_MOCTR62 */
    { 0xF00057ECu, 0x3FFFFFFFu },  /* CAN_MOAMR63 */
    { 0xF00057FCu, 0x403E0000u },  /* CAN_MOCTR63 */
    { 0xF000580Cu, 0x3FFFFFFFu },  /* CAN_MOAMR64 */
    { 0xF000581Cu, 0x413F0000u },  /* CAN_MOCTR64 */
    { 0xF000582Cu, 0x3FFFFFFFu },  /* CAN_MOAMR65 */
    { 0xF000583Cu, 0x42400000u },  /* CAN_MOCTR65 */
    { 0xF000584Cu, 0x3FFFFFFFu },  /* CAN_MOAMR66 */
    { 0xF000585Cu, 0x43410000u },  /* CAN_MOCTR66 */
    { 0xF000586Cu, 0x3FFFFFFFu },  /* CAN_MOAMR67 */
    { 0xF000587Cu, 0x44420000u },  /* CAN_MOCTR67 */
    { 0xF000588Cu, 0x3FFFFFFFu },  /* CAN_MOAMR68 */
    { 0xF000589Cu, 0x45430000u },  /* CAN_MOCTR68 */
    { 0xF00058ACu, 0x3FFFFFFFu },  /* CAN_MOAMR69 */
    { 0xF00058BCu, 0x46440000u },  /* CAN_MOCTR69 */
    { 0xF00058CCu, 0x3FFFFFFFu },  /* CAN_MOAMR70 */
    { 0xF00058DCu, 0x47450000u },  /* CAN_MOCTR70 */
    { 0xF00058ECu, 0x3FFFFFFFu },  /* CAN_MOAMR71 */
    { 0xF00058FCu, 0x48460000u },  /* CAN_MOCTR71 */
    { 0xF000590Cu, 0x3FFFFFFFu },  /* CAN_MOAMR72 */
    { 0xF000591Cu, 0x49470000u },  /* CAN_MOCTR72 */
    { 0xF000592Cu, 0x3FFFFFFFu },  /* CAN_MOAMR73 */
    { 0xF000593Cu, 0x4A480000u },  /* CAN_MOCTR73 */
    { 0xF000594Cu, 0x3FFFFFFFu },  /* CAN_MOAMR74 */
    { 0xF000595Cu, 0x4B490000u },  /* CAN_MOCTR74 */
    { 0xF000596Cu, 0x3FFFFFFFu },  /* CAN_MOAMR75 */
    { 0xF000597Cu, 0x4C4A0000u },  /* CAN_MOCTR75 */
    { 0xF000598Cu, 0x3FFFFFFFu },  /* CAN_MOAMR76 */
    { 0xF000599Cu, 0x4D4B0000u },  /* CAN_MOCTR76 */
    { 0xF00059ACu, 0x3FFFFFFFu },  /* CAN_MOAMR77 */
    { 0xF00059BCu, 0x4E4C0000u },  /* CAN_MOCTR77 */
    { 0xF00059CCu, 0x3FFFFFFFu },  /* CAN_MOAMR78 */
    { 0xF00059DCu, 0x4F4D0000u },  /* CAN_MOCTR78 */
    { 0xF00059ECu, 0x3FFFFFFFu },  /* CAN_MOAMR79 */
    { 0xF00059FCu, 0x504E0000u },  /* CAN_MOCTR79 */
    { 0xF0005A0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR80 */
    { 0xF0005A1Cu, 0x514F0000u },  /* CAN_MOCTR80 */
    { 0xF0005A2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR81 */
    { 0xF0005A3Cu, 0x52500000u },  /* CAN_MOCTR81 */
    { 0xF0005A4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR82 */
    { 0xF0005A5Cu, 0x53510000u },  /* CAN_MOCTR82 */
    { 0xF0005A6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR83 */
    { 0xF0005A7Cu, 0x54520000u },  /* CAN_MOCTR83 */
    { 0xF0005A8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR84 */
    { 0xF0005A9Cu, 0x55530000u },  /* CAN_MOCTR84 */
    { 0xF0005AACu, 0x3FFFFFFFu },  /* CAN_MOAMR85 */
    { 0xF0005ABCu, 0x56540000u },  /* CAN_MOCTR85 */
    { 0xF0005ACCu, 0x3FFFFFFFu },  /* CAN_MOAMR86 */
    { 0xF0005ADCu, 0x57550000u },  /* CAN_MOCTR86 */
    { 0xF0005AECu, 0x3FFFFFFFu },  /* CAN_MOAMR87 */
    { 0xF0005AFCu, 0x58560000u },  /* CAN_MOCTR87 */
    { 0xF0005B0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR88 */
    { 0xF0005B1Cu, 0x59570000u },  /* CAN_MOCTR88 */
    { 0xF0005B2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR89 */
    { 0xF0005B3Cu, 0x5A580000u },  /* CAN_MOCTR89 */
    { 0xF0005B4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR90 */
    { 0xF0005B5Cu, 0x5B590000u },  /* CAN_MOCTR90 */
    { 0xF0005B6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR91 */
    { 0xF0005B7Cu, 0x5C5A0000u },  /* CAN_MOCTR91 */
    { 0xF0005B8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR92 */
    { 0xF0005B9Cu, 0x5D5B0000u },  /* CAN_MOCTR92 */
    { 0xF0005BACu, 0x3FFFFFFFu },  /* CAN_MOAMR93 */
    { 0xF0005BBCu, 0x5E5C0000u },  /* CAN_MOCTR93 */
    { 0xF0005BCCu, 0x3FFFFFFFu },  /* CAN_MOAMR94 */
    { 0xF0005BDCu, 0x5F5D0000u },  /* CAN_MOCTR94 */
    { 0xF0005BECu, 0x3FFFFFFFu },  /* CAN_MOAMR95 */
    { 0xF0005BFCu, 0x605E0000u },  /* CAN_MOCTR95 */
    { 0xF0005C0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR96 */
    { 0xF0005C1Cu, 0x615F0000u },  /* CAN_MOCTR96 */
    { 0xF0005C2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR97 */
    { 0xF0005C3Cu, 0x62600000u },  /* CAN_MOCTR97 */
    { 0xF0005C4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR98 */
    { 0xF0005C5Cu, 0x63610000u },  /* CAN_MOCTR98 */
    { 0xF0005C6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR99 */
    { 0xF0005C7Cu, 0x64620000u },  /* CAN_MOCTR99 */
    { 0xF0005C8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR100 */
    { 0xF0005C9Cu, 0x65630000u },  /* CAN_MOCTR100 */
    { 0xF0005CACu, 0x3FFFFFFFu },  /* CAN_MOAMR101 */
    { 0xF0005CBCu, 0x66640000u },  /* CAN_MOCTR101 */
    { 0xF0005CCCu, 0x3FFFFFFFu },  /* CAN_MOAMR102 */
    { 0xF0005CDCu, 0x67650000u },  /* CAN_MOCTR102 */
    { 0xF0005CECu, 0x3FFFFFFFu },  /* CAN_MOAMR103 */
    { 0xF0005CFCu, 0x68660000u },  /* CAN_MOCTR103 */
    { 0xF0005D0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR104 */
    { 0xF0005D1Cu, 0x69670000u },  /* CAN_MOCTR104 */
    { 0xF0005D2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR105 */
    { 0xF0005D3Cu, 0x6A680000u },  /* CAN_MOCTR105 */
    { 0xF0005D4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR106 */
    { 0xF0005D5Cu, 0x6B690000u },  /* CAN_MOCTR106 */
    { 0xF0005D6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR107 */
    { 0xF0005D7Cu, 0x6C6A0000u },  /* CAN_MOCTR107 */
    { 0xF0005D8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR108 */
    { 0xF0005D9Cu, 0x6D6B0000u },  /* CAN_MOCTR108 */
    { 0xF0005DACu, 0x3FFFFFFFu },  /* CAN_MOAMR109 */
    { 0xF0005DBCu, 0x6E6C0000u },  /* CAN_MOCTR109 */
    { 0xF0005DCCu, 0x3FFFFFFFu },  /* CAN_MOAMR110 */
    { 0xF0005DDCu, 0x6F6D0000u },  /* CAN_MOCTR110 */
    { 0xF0005DECu, 0x3FFFFFFFu },  /* CAN_MOAMR111 */
    { 0xF0005DFCu, 0x706E0000u },  /* CAN_MOCTR111 */
    { 0xF0005E0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR112 */
    { 0xF0005E1Cu, 0x716F0000u },  /* CAN_MOCTR112 */
    { 0xF0005E2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR113 */
    { 0xF0005E3Cu, 0x72700000u },  /* CAN_MOCTR113 */
    { 0xF0005E4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR114 */
    { 0xF0005E5Cu, 0x73710000u },  /* CAN_MOCTR114 */
    { 0xF0005E6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR115 */
    { 0xF0005E7Cu, 0x74720000u },  /* CAN_MOCTR115 */
    { 0xF0005E8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR116 */
    { 0xF0005E9Cu, 0x75730000u },  /* CAN_MOCTR116 */
    { 0xF0005EACu, 0x3FFFFFFFu },  /* CAN_MOAMR117 */
    { 0xF0005EBCu, 0x76740000u },  /* CAN_MOCTR117 */
    { 0xF0005ECCu, 0x3FFFFFFFu },  /* CAN_MOAMR118 */
    { 0xF0005EDCu, 0x77750000u },  /* CAN_MOCTR118 */
    { 0xF0005EECu, 0x3FFFFFFFu },  /* CAN_MOAMR119 */
    { 0xF0005EFCu, 0x78760000u },  /* CAN_MOCTR119 */
    { 0xF0005F0Cu, 0x3FFFFFFFu },  /* CAN_MOAMR120 */
    { 0xF0005F1Cu, 0x79770000u },  /* CAN_MOCTR120 */
    { 0xF0005F2Cu, 0x3FFFFFFFu },  /* CAN_MOAMR121 */
    { 0xF0005F3Cu, 0x7A780000u },  /* CAN_MOCTR121 */
    { 0xF0005F4Cu, 0x3FFFFFFFu },  /* CAN_MOAMR122 */
    { 0xF0005F5Cu, 0x7B790000u },  /* CAN_MOCTR122 */
    { 0xF0005F6Cu, 0x3FFFFFFFu },  /* CAN_MOAMR123 */
    { 0xF0005F7Cu, 0x7C7A0000u },  /* CAN_MOCTR123 */
    { 0xF0005F8Cu, 0x3FFFFFFFu },  /* CAN_MOAMR124 */
    { 0xF0005F9Cu, 0x7D7B0000u },  /* CAN_MOCTR124 */
    { 0xF0005FACu, 0x3FFFFFFFu },  /* CAN_MOAMR125 */
    { 0xF0005FBCu, 0x7E7C0000u },  /* CAN_MOCTR125 */
    { 0xF0005FCCu, 0x3FFFFFFFu },  /* CAN_MOAMR126 */
    { 0xF0005FDCu, 0x7F7D0000u },  /* CAN_MOCTR126 */
    { 0xF0005FECu, 0x3FFFFFFFu },  /* CAN_MOAMR127 */
    { 0xF0005FFCu, 0x7F7E0000u },  /* CAN_MOCTR127 */
    /* ---- ERAY ---- */
    { 0xF0010000u, 0x00000100u },  /* ERAY_CLC */
    { 0xF0010008u, 0x0044C000u },  /* ERAY_ID (rev nibble=0) */
    { 0xF0010010u, 0x00000300u },  /* ERAY_TEST1 */
    { 0xF001002Cu, 0x0303FFFFu },  /* ERAY_SILS */
    { 0xF0010048u, 0x00020000u },  /* ERAY_T1C */
    { 0xF0010080u, 0x0C401080u },  /* ERAY_SUCC1 */
    { 0xF0010084u, 0x01000504u },  /* ERAY_SUCC2 */
    { 0xF0010088u, 0x00000011u },  /* ERAY_SUCC3 */
    { 0xF0010090u, 0x084C0633u },  /* ERAY_PRTC1 */
    { 0xF0010094u, 0x0F2D0A0Eu },  /* ERAY_PRTC2 */
    { 0xF00100A0u, 0x00000280u },  /* ERAY_GTUC01 */
    { 0xF00100A4u, 0x0002000Au },  /* ERAY_GTUC02 */
    { 0xF00100A8u, 0x02020000u },  /* ERAY_GTUC03 */
    { 0xF00100ACu, 0x00080007u },  /* ERAY_GTUC04 */
    { 0xF00100B0u, 0x0E000000u },  /* ERAY_GTUC05 */
    { 0xF00100B4u, 0x00020000u },  /* ERAY_GTUC06 */
    { 0xF00100B8u, 0x00020004u },  /* ERAY_GTUC07 */
    { 0xF00100BCu, 0x00000002u },  /* ERAY_GTUC08 */
    { 0xF00100C0u, 0x00000101u },  /* ERAY_GTUC09 */
    { 0xF00100C4u, 0x00020005u },  /* ERAY_GTUC10 */
    { 0xF0010100u, 0x00104000u },  /* ERAY_CCSV */
    { 0xF0010300u, 0x01800000u },  /* ERAY_MRC */
    { 0xF0010304u, 0x01800000u },  /* ERAY_FRF */
    { 0xF001030Cu, 0x00000080u },  /* ERAY_FCL */
    { 0xF0010310u, 0x00000080u },  /* ERAY_MHDS */
    { 0xF00103F4u, 0x87654321u },  /* ERAY_ENDN */
    /* ---- PCP ---- */
    { 0xF0043F08u, 0x0020C000u },  /* PCP_ID (rev nibble=0) */
    { 0xF0043F28u, 0x000003E4u },  /* PCP_ICON */
    { 0xF0043FD0u, 0x00001400u },  /* PCP_SRC11 */
    { 0xF0043FD4u, 0x00001400u },  /* PCP_SRC10 */
    { 0xF0043FD8u, 0x00001400u },  /* PCP_SRC9 */
    { 0xF0043FDCu, 0x00001000u },  /* PCP_SRC8 */
    { 0xF0043FE0u, 0x00001000u },  /* PCP_SRC7 */
    { 0xF0043FE4u, 0x00001000u },  /* PCP_SRC6 */
    { 0xF0043FE8u, 0x00001000u },  /* PCP_SRC5 */
    { 0xF0043FECu, 0x00001000u },  /* PCP_SRC4 */
    { 0xF0043FF0u, 0x00001400u },  /* PCP_SRC3 */
    { 0xF0043FF4u, 0x00001400u },  /* PCP_SRC2 */
    { 0xF0043FF8u, 0x00001000u },  /* PCP_SRC1 */
    { 0xF0043FFCu, 0x00001000u },  /* PCP_SRC0 */
    /* ---- SSC ---- */
    { 0xF0100100u, 0x00000003u },  /* SSC0_CLC */
    { 0xF0100108u, 0x00004500u },  /* SSC0_ID (rev nibble=0) */
    { 0xF0100200u, 0x00000003u },  /* SSC1_CLC */
    { 0xF0100208u, 0x00004500u },  /* SSC1_ID (rev nibble=0) */
    /* ---- FADC ---- */
    { 0xF0100400u, 0x00000003u },  /* FADC_CLC */
    { 0xF0100454u, 0x000000E4u },  /* FADC_ALR */
    /* ---- ADC ---- */
    { 0xF0101000u, 0x00000003u },  /* ADC0_CLC */
    { 0xF0101030u, 0x000000FFu },  /* ADC0_GLOBCTR */
    { 0xF0101084u, 0x00000020u },  /* ADC0_QSR0 */
    { 0xF01010A4u, 0x00000020u },  /* ADC0_QSR2 */
    { 0xF01010C4u, 0x00000020u },  /* ADC0_QSR4 */
    { 0xF01010F0u, 0x00000198u },  /* ADC0_LCBR0 */
    { 0xF01010F4u, 0x00000E64u },  /* ADC0_LCBR1 */
    { 0xF01010F8u, 0x00000554u },  /* ADC0_LCBR2 */
    { 0xF01010FCu, 0x00000AA8u },  /* ADC0_LCBR3 */
    { 0xF0101210u, 0x00000100u },  /* ADC0_ALR0 */
    { 0xF0101430u, 0x000000FFu },  /* ADC1_GLOBCTR */
    { 0xF0101484u, 0x00000020u },  /* ADC1_QSR0 */
    { 0xF01014A4u, 0x00000020u },  /* ADC1_QSR2 */
    { 0xF01014C4u, 0x00000020u },  /* ADC1_QSR4 */
    { 0xF01014F0u, 0x00000198u },  /* ADC1_LCBR0 */
    { 0xF01014F4u, 0x00000E64u },  /* ADC1_LCBR1 */
    { 0xF01014F8u, 0x00000554u },  /* ADC1_LCBR2 */
    { 0xF01014FCu, 0x00000AA8u },  /* ADC1_LCBR3 */
    { 0xF0101610u, 0x00000100u },  /* ADC1_ALR0 */
    { 0xF0101830u, 0x000000FFu },  /* ADC2_GLOBCTR */
    { 0xF0101884u, 0x00000020u },  /* ADC2_QSR0 */
    { 0xF01018A4u, 0x00000020u },  /* ADC2_QSR2 */
    { 0xF01018C4u, 0x00000020u },  /* ADC2_QSR4 */
    { 0xF01018F0u, 0x00000198u },  /* ADC2_LCBR0 */
    { 0xF01018F4u, 0x00000E64u },  /* ADC2_LCBR1 */
    { 0xF01018F8u, 0x00000554u },  /* ADC2_LCBR2 */
    { 0xF01018FCu, 0x00000AA8u },  /* ADC2_LCBR3 */
    { 0xF0101A10u, 0x00000100u },  /* ADC2_ALR0 */
    /* ---- MLI ---- */
    { 0xF010C008u, 0x0025C007u },  /* MLI0_ID */
    { 0xF010C00Cu, 0x03FF43FFu },  /* MLI0_FDR */
    { 0xF010C010u, 0x00000110u },  /* MLI0_TCR */
    { 0xF010C068u, 0x01000000u },  /* MLI0_RCR */
    { 0xF010C0B4u, 0x10008000u },  /* MLI0_OICR */
    { 0xF010C108u, 0x0025C007u },  /* MLI1_ID */
    { 0xF010C10Cu, 0x03FF43FFu },  /* MLI1_FDR */
    { 0xF010C110u, 0x00000110u },  /* MLI1_TCR */
    { 0xF010C168u, 0x01000000u },  /* MLI1_RCR */
    { 0xF010C1B4u, 0x10008000u },  /* MLI1_OICR */
    /* ---- DMA ---- */
    { 0xF010C208u, 0x001BC001u },  /* MCHK_ID */
    /* ---- EBU ---- */
    { 0xF8000000u, 0x00010000u },  /* EBU_CLC */
    { 0xF8000004u, 0x00000020u },  /* EBU_MODCON */
    { 0xF8000010u, 0x00000001u },  /* EBU_EXTBOOT */
    { 0xF8000018u, 0xA0000001u },  /* EBU_ADDRSEL0 */
    { 0xF8000028u, 0x00D30040u },  /* EBU_BUSRCON0 */
    { 0xF800002Cu, 0xFFFFFFFFu },  /* EBU_BUSRAP0 */
    { 0xF8000030u, 0x00D30000u },  /* EBU_BUSWCON0 */
    { 0xF8000034u, 0xFFFFFFFFu },  /* EBU_BUSWAP0 */
    { 0xF8000038u, 0x00D30040u },  /* EBU_BUSRCON1 */
    { 0xF800003Cu, 0xFFFFFFFFu },  /* EBU_BUSRAP1 */
    { 0xF8000040u, 0x00D30000u },  /* EBU_BUSWCON1 */
    { 0xF8000044u, 0xFFFFFFFFu },  /* EBU_BUSWAP1 */
    { 0xF8000048u, 0x00D30040u },  /* EBU_BUSRCON2 */
    { 0xF800004Cu, 0xFFFFFFFFu },  /* EBU_BUSRAP2 */
    { 0xF8000050u, 0x00D30000u },  /* EBU_BUSWCON2 */
    { 0xF8000054u, 0xFFFFFFFFu },  /* EBU_BUSWAP2 */
    { 0xF8000058u, 0x00D30040u },  /* EBU_BUSRCON3 */
    { 0xF800005Cu, 0xFFFFFFFFu },  /* EBU_BUSRAP3 */
    { 0xF8000060u, 0x00D30000u },  /* EBU_BUSWCON3 */
    { 0xF8000064u, 0xFFFFFFFFu },  /* EBU_BUSWAP3 */
    /* ---- PMU ---- */
    { 0xF8000508u, 0x0050C000u },  /* PMU0_ID (rev nibble=0) */
    { 0xF8000608u, 0x0051C000u },  /* PMU1_ID (rev nibble=0) */
    { 0xF8002008u, 0x0053C000u },  /* FLASH0_ID (rev nibble=0) */
    { 0xF8002014u, 0x00070606u },  /* FLASH0_FCON */
    { 0xF8002018u, 0x00008000u },  /* FLASH0_MARP */
    { 0xF800201Cu, 0x00008000u },  /* FLASH0_MARD */
    { 0xF8004008u, 0x0055C000u },  /* FLASH1_ID (rev nibble=0) */
    { 0xF8004014u, 0x00070606u },  /* FLASH1_FCON */
    { 0xF8004018u, 0x00008000u },  /* FLASH1_MARP */
    { 0xF800401Cu, 0x00008000u },  /* FLASH1_MARD */
};
#define TC1797_RESET_SEED_N (sizeof(tc1797_reset_seed)/sizeof(tc1797_reset_seed[0]))
#endif
