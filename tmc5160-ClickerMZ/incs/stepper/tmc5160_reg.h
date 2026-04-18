#ifndef TMC5160_REG_H
#define TMC5160_REG_H

/* ============================================================
 * TMC5160 Register Map
 * All addresses are 7-bit (bit7 = 0:read, 1:write in SPI frame)
 * ============================================================ */

/* --- General Configuration --- */
#define TMC5160_REG_GCONF           0x00U
#define TMC5160_REG_GSTAT           0x01U
#define TMC5160_REG_IFCNT           0x02U
#define TMC5160_REG_NODECONF        0x03U
#define TMC5160_REG_IOIN            0x04U
#define TMC5160_REG_OUTPUT          0x05U
#define TMC5160_REG_X_COMPARE       0x05U

/* --- Velocity Dependent Driver Feature Control --- */
#define TMC5160_REG_IHOLD_IRUN      0x10U
#define TMC5160_REG_TPOWERDOWN      0x11U
#define TMC5160_REG_TSTEP           0x12U
#define TMC5160_REG_TPWMTHRS        0x13U
#define TMC5160_REG_TCOOLTHRS       0x14U
#define TMC5160_REG_THIGH           0x15U

/* --- Ramp Generator Motion Control --- */
#define TMC5160_REG_RAMPMODE        0x20U
#define TMC5160_REG_XACTUAL         0x21U
#define TMC5160_REG_VACTUAL         0x22U
#define TMC5160_REG_VSTART          0x23U
#define TMC5160_REG_A1              0x24U
#define TMC5160_REG_V1              0x25U
#define TMC5160_REG_AMAX            0x26U
#define TMC5160_REG_VMAX            0x27U
#define TMC5160_REG_DMAX            0x28U
#define TMC5160_REG_D1              0x2AU
#define TMC5160_REG_VSTOP           0x2BU
#define TMC5160_REG_TZEROWAIT       0x2CU
#define TMC5160_REG_XTARGET         0x2DU

/* --- Ramp Generator Driver Feature Control --- */
#define TMC5160_REG_VDCMIN          0x33U
#define TMC5160_REG_SW_MODE         0x34U
#define TMC5160_REG_RAMP_STAT       0x35U
#define TMC5160_REG_XLATCH          0x36U

/* --- Encoder --- */
#define TMC5160_REG_ENCMODE         0x38U
#define TMC5160_REG_X_ENC           0x39U
#define TMC5160_REG_ENC_CONST       0x3AU
#define TMC5160_REG_ENC_STATUS      0x3BU
#define TMC5160_REG_ENC_LATCH       0x3CU

/* --- Motor Driver --- */
#define TMC5160_REG_MSLUT0          0x60U
#define TMC5160_REG_MSLUTSEL        0x68U
#define TMC5160_REG_MSLUTSTART      0x69U
#define TMC5160_REG_MSCNT           0x6AU
#define TMC5160_REG_MSCURACT        0x6BU
#define TMC5160_REG_CHOPCONF        0x6CU
#define TMC5160_REG_COOLCONF        0x6DU
#define TMC5160_REG_DCCTRL          0x6EU
#define TMC5160_REG_DRVSTATUS       0x6FU
#define TMC5160_REG_PWMCONF         0x70U
#define TMC5160_REG_PWM_SCALE       0x71U
#define TMC5160_REG_PWM_AUTO        0x72U
#define TMC5160_REG_LOST_STEPS      0x73U

/* ============================================================
 * RAMPMODE values
 * ============================================================ */
#define TMC5160_RAMPMODE_POSITION   0x00U  /* positioning mode */
#define TMC5160_RAMPMODE_VEL_POS    0x01U  /* velocity mode +direction */
#define TMC5160_RAMPMODE_VEL_NEG    0x02U  /* velocity mode -direction */
#define TMC5160_RAMPMODE_HOLD       0x03U  /* hold position, no ramp */

/* ============================================================
 * GCONF bit masks
 * ============================================================ */
#define TMC5160_GCONF_RECALIBRATE       (1UL << 0)
#define TMC5160_GCONF_FASTSTANDSTILL    (1UL << 1)
#define TMC5160_GCONF_EN_PWM_MODE       (1UL << 2)   /* StealthChop */
#define TMC5160_GCONF_MULTISTEP_FILT    (1UL << 3)
#define TMC5160_GCONF_SHAFT             (1UL << 4)   /* invert direction */
#define TMC5160_GCONF_DIAG0_ERROR       (1UL << 5)
#define TMC5160_GCONF_DIAG1_STALL       (1UL << 8)   /* DIAG1 = stall flag */
#define TMC5160_GCONF_DIAG1_POSCOMP     (1UL << 12)
#define TMC5160_GCONF_STOP_ENABLE       (1UL << 15)

/* ============================================================
 * CHOPCONF bit masks (key ones)
 * ============================================================ */
#define TMC5160_CHOPCONF_TOFF_MASK      0x0FU        /* off time */
#define TMC5160_CHOPCONF_TBL_SHIFT      15U
#define TMC5160_CHOPCONF_VSENSE         (1UL << 17)
#define TMC5160_CHOPCONF_MRES_SHIFT     24U          /* microstep resolution */
#define TMC5160_CHOPCONF_MRES_MASK      (0x0FUL << 24)
#define TMC5160_CHOPCONF_INTPOL         (1UL << 28)  /* interpolate to 256 */

/* ============================================================
 * IHOLD_IRUN field positions
 * ============================================================ */
#define TMC5160_IHOLD_SHIFT         0U
#define TMC5160_IRUN_SHIFT          8U
#define TMC5160_IHOLDDELAY_SHIFT    16U

/* ============================================================
 * DRV_STATUS bit masks
 * ============================================================ */
#define TMC5160_DRVSTATUS_SG_RESULT_MASK    0x3FFU       /* bits 0-9 */
#define TMC5160_DRVSTATUS_FSACTIVE          (1UL << 15)
#define TMC5160_DRVSTATUS_STALLGUARD        (1UL << 24)
#define TMC5160_DRVSTATUS_OT                (1UL << 25)  /* overtemp */
#define TMC5160_DRVSTATUS_OTPW              (1UL << 26)  /* overtemp warning */
#define TMC5160_DRVSTATUS_S2GA              (1UL << 27)
#define TMC5160_DRVSTATUS_S2GB              (1UL << 28)
#define TMC5160_DRVSTATUS_OLA               (1UL << 29)  /* open load A */
#define TMC5160_DRVSTATUS_OLB               (1UL << 30)  /* open load B */
#define TMC5160_DRVSTATUS_STST              (1UL << 31)  /* standstill */

/* ============================================================
 * RAMP_STAT bit masks
 * ============================================================ */
#define TMC5160_RAMPSTAT_VEL_REACHED        (1UL << 8)
#define TMC5160_RAMPSTAT_POS_REACHED        (1UL << 9)

/* ============================================================
 * COOLCONF (StallGuard/CoolStep) field positions
 * ============================================================ */
#define TMC5160_COOLCONF_SEMIN_SHIFT        0U
#define TMC5160_COOLCONF_SEUP_SHIFT         5U
#define TMC5160_COOLCONF_SEMAX_SHIFT        8U
#define TMC5160_COOLCONF_SEDN_SHIFT         13U
#define TMC5160_COOLCONF_SEIMIN             (1UL << 15)
#define TMC5160_COOLCONF_SGT_SHIFT          16U   /* StallGuard threshold -64..63 */
#define TMC5160_COOLCONF_SFILT              (1UL << 24)

/* ============================================================
 * SPI write flag
 * ============================================================ */
#define TMC5160_WRITE_FLAG              0x80U

#endif /* TMC5160_REG_H */