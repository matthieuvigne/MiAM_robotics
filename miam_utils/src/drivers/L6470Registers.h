// Enums describing L6470 registers and parameters.

typedef enum DSPIN_OVERCURRENT_CONST
{
    dSPIN_OCD_TH_375mA  = 0x00,
    dSPIN_OCD_TH_750mA  = 0x01,
    dSPIN_OCD_TH_1125mA = 0x02,
    dSPIN_OCD_TH_1500mA = 0x03,
    dSPIN_OCD_TH_1875mA = 0x04,
    dSPIN_OCD_TH_2250mA = 0x05,
    dSPIN_OCD_TH_2625mA = 0x06,
    dSPIN_OCD_TH_3000mA = 0x07,
    dSPIN_OCD_TH_3375mA = 0x08,
    dSPIN_OCD_TH_3750mA = 0x09,
    dSPIN_OCD_TH_4125mA = 0x0A,
    dSPIN_OCD_TH_4500mA = 0x0B,
    dSPIN_OCD_TH_4875mA = 0x0C,
    dSPIN_OCD_TH_5250mA = 0x0D,
    dSPIN_OCD_TH_5625mA = 0x0E,
    dSPIN_OCD_TH_6000mA = 0x0F

} DSPIN_OVERCURRENT_CONST;

// Default value of the config register.
#define dSPIN_DEFAULT_CONFIG 11912

double STEPSEC_TO_VELOCITY_REG = 250e-9 * (1<<28); // Conversion ratio from step/s to velocity register value.

typedef enum DSPIN_REG_CONST
{
    //  Define the SYNC_EN bit. When set, the BUSYN pin will instead
    //  output a clock related to the full-step frequency as defined by the
    //  SYNC_SEL bits below.
    //
    dSPIN_STEP_MODE_SYNC_EN        = 0x80,  // Mask for this bit
    dSPIN_SYNC_EN               = 0x80,

    //  Define the SYNC_SEL modes. The clock output is defined by
    //  the full-step frequency and the value in these bits- see the datasheet
    //  for a matrix describing that relationship (page 46).
    //
    dSPIN_STEP_MODE_SYNC_SEL    = 0x70,
    dSPIN_SYNC_SEL_1_2          = 0x00,
    dSPIN_SYNC_SEL_1            = 0x10,
    dSPIN_SYNC_SEL_2            = 0x20,
    dSPIN_SYNC_SEL_4            = 0x30,
    dSPIN_SYNC_SEL_8            = 0x40,
    dSPIN_SYNC_SEL_16           = 0x50,
    dSPIN_SYNC_SEL_32           = 0x60,
    dSPIN_SYNC_SEL_64           = 0x70,

    // Bit names for the ALARM_EN register.
    // Each of these bits defines one potential alarm condition.
    // When one of these conditions occurs and the respective bit in ALARM_EN
    // is set, the FLAG pin will go low. The register must be queried to
    // determine which event caused the alarm.
    //
    dSPIN_ALARM_EN_OVERCURRENT       = 0x01,
    dSPIN_ALARM_EN_THERMAL_SHUTDOWN     = 0x02,
    dSPIN_ALARM_EN_THERMAL_WARNING   = 0x04,
    dSPIN_ALARM_EN_UNDER_VOLTAGE     = 0x08,
    dSPIN_ALARM_EN_STALL_DET_A       = 0x10,
    dSPIN_ALARM_EN_STALL_DET_B       = 0x20,
    dSPIN_ALARM_EN_SW_TURN_ON        = 0x40,
    dSPIN_ALARM_EN_WRONG_NPERF_CMD   = 0x80,

    // Oscillator options.
    //
    // The dSPIN needs to know what the clock frequency is because it uses
    // that for some calculations during operation.
    //
    dSPIN_CONFIG_OSC_SEL                 = 0x000F, // Mask for this bit field.
    dSPIN_CONFIG_INT_16MHZ               = 0x0000, // Int 16MHz, no output
    dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = 0x0008, // Int 16MHz, 2MHz output
    dSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = 0x0009, // Int 16MHz, 4MHz output
    dSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = 0x000A, // Int 16MHz, 8MHz output
    dSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = 0x000B, // Int 16MHz, 16MHz output
    dSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE     = 0x0004, // Ext 8MHz crystal
    dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE    = 0x0005, // Ext 16MHz crystal
    dSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE    = 0x0006, // Ext 24MHz crystal
    dSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE    = 0x0007, // Ext 32MHz crystal
    dSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = 0x000C, // Ext 8MHz driven, out inv
    dSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT = 0x000D, // Ext 16MHz driven, out inv
    dSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT = 0x000E, // Ext 24MHz driven, out inv
    dSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT = 0x000F, // Ext 32MHz driven, out inv

    // Configure the functionality of the external switch input
    //
    dSPIN_CONFIG_SW_MODE                 = 0x0010, // Mask for this bit.
    dSPIN_CONFIG_SW_HARD_STOP            = 0x0000, // Hard stop motor on switch.
    dSPIN_CONFIG_SW_USER                 = 0x0010, // Tie to the GoUntil and ReleaseSW
                                                   // commands to provide jog function.
                                                   // See page 25 of datasheet.

    // Configure the motor voltage compensation mode (see page 34 of datasheet)
    //
    dSPIN_CONFIG_EN_VSCOMP               = 0x0020,  // Mask for this bit.
    dSPIN_CONFIG_VS_COMP_DISABLE         = 0x0000,  // Disable voltage comp.
    dSPIN_CONFIG_VS_COMP_ENABLE          = 0x0020,  // Enable voltage comp.

    // Configure overcurrent detection event handling
    //
    dSPIN_CONFIG_OC_SD                   = 0x0080,  // Mask for this bit.
    dSPIN_CONFIG_OC_SD_DISABLE           = 0x0000,  // Bridges do NOT shutdown on OC detect
    dSPIN_CONFIG_OC_SD_ENABLE            = 0x0080,  // Bridges shutdown on OC detect

    // Configure the slew rate of the power bridge output
    //
    dSPIN_CONFIG_POW_SR                  = 0x0300,  // Mask for this bit field.
    dSPIN_CONFIG_SR_180V_us              = 0x0000,  // 180V/us
    dSPIN_CONFIG_SR_290V_us              = 0x0200,  // 290V/us
    dSPIN_CONFIG_SR_530V_us              = 0x0300,  // 530V/us

    // Integer divisors for PWM sinewave generation
    // See page 32 of the datasheet for more information on this.
    //
    dSPIN_CONFIG_F_PWM_DEC               = 0x1C00,  // mask for this bit field
    dSPIN_CONFIG_PWM_MUL_0_625           = (0x00)<<10,
    dSPIN_CONFIG_PWM_MUL_0_75            = (0x01)<<10,
    dSPIN_CONFIG_PWM_MUL_0_875           = (0x02)<<10,
    dSPIN_CONFIG_PWM_MUL_1               = (0x03)<<10,
    dSPIN_CONFIG_PWM_MUL_1_25            = (0x04)<<10,
    dSPIN_CONFIG_PWM_MUL_1_5             = (0x05)<<10,
    dSPIN_CONFIG_PWM_MUL_1_75            = (0x06)<<10,
    dSPIN_CONFIG_PWM_MUL_2               = (0x07)<<10,

    // Multiplier for the PWM sinewave frequency
    //
    dSPIN_CONFIG_F_PWM_INT               = 0xE000,  // mask for this bit field.
    dSPIN_CONFIG_PWM_DIV_1               = (0x00)<<13,
    dSPIN_CONFIG_PWM_DIV_2               = (0x01)<<13,
    dSPIN_CONFIG_PWM_DIV_3               = (0x02)<<13,
    dSPIN_CONFIG_PWM_DIV_4               = (0x03)<<13,
    dSPIN_CONFIG_PWM_DIV_5               = (0x04)<<13,
    dSPIN_CONFIG_PWM_DIV_6               = (0x05)<<13,
    dSPIN_CONFIG_PWM_DIV_7               = (0x06)<<13,

    // Status register bit renames:
    // Read-only bits conferring information about the device to the user.
    //
    dSPIN_STATUS_HIZ                     = 0x0001, // 1 when bridges are HiZ.
    dSPIN_STATUS_BUSY                    = 0x0002, // Mirrors BUSY pin.
    dSPIN_STATUS_SW_F                    = 0x0004, // 0 - switch open, 1 - closed
    dSPIN_STATUS_SW_EVN                  = 0x0008, // active high, set on switch
                                                   // falling edge, cleared by
                                                   // reading STATUS.
    dSPIN_STATUS_DIR                     = 0x0010, // Indicates current motor direction.
                                                   // 1 - FWD, 0 - REV.
    dSPIN_STATUS_NOTPERF_CMD             = 0x0080, // Last command not performed.
    dSPIN_STATUS_WRONG_CMD               = 0x0100, // Last command not valid.
    dSPIN_STATUS_UVLO                    = 0x0200, // Undervoltage lockout active.
    dSPIN_STATUS_TH_WRN                  = 0x0400, // Thermal warning
    dSPIN_STATUS_TH_SD                   = 0x0800, // Thermal shutdown
    dSPIN_STATUS_OCD                     = 0x1000, // Overcurrent detected
    dSPIN_STATUS_STEP_LOSS_A             = 0x2000, // Stall detected on A bridge
    dSPIN_STATUS_STEP_LOSS_B             = 0x4000, // Stall detected on B bridge
    dSPIN_STATUS_SCK_MOD                 = 0x8000, // Step clock mode is active

    // Status register motor status field
    //
    dSPIN_STATUS_MOT_STATUS               = 0x0060,       // field mask
    dSPIN_STATUS_MOT_STATUS_STOPPED       = (0x0000)<<13, // Motor stopped
    dSPIN_STATUS_MOT_STATUS_ACCELERATION  = (0x0001)<<13, // Motor accelerating
    dSPIN_STATUS_MOT_STATUS_DECELERATION  = (0x0002)<<13, // Motor decelerating
    dSPIN_STATUS_MOT_STATUS_CONST_SPD     = (0x0003)<<13, // Motor at speed

    /* dSPIN direction options */
    dSPIN_FWD  = 0x01,
    dSPIN_REV  = 0x00,

    /* dSPIN action options */
    dSPIN_ACTION_RESET  = 0x00,
    dSPIN_ACTION_COPY   = 0x01

} DSPIN_REG_CONST;

typedef enum dSPIN_COMMANDS
{
    dSPIN_NOP                  = 0x00,
    dSPIN_SET_PARAM            = 0x00,
    dSPIN_GET_PARAM            = 0x20,
    dSPIN_RUN                  = 0x50,
    dSPIN_STEP_CLOCK           = 0x58,
    dSPIN_MOVE                 = 0x40,
    dSPIN_GOTO                 = 0x60,
    dSPIN_GOTO_DIR             = 0x68,
    dSPIN_GO_UNTIL             = 0x82,
    dSPIN_RELEASE_SW           = 0x92,
    dSPIN_GO_HOME              = 0x70,
    dSPIN_GO_MARK              = 0x78,
    dSPIN_RESET_POS            = 0xD8,
    dSPIN_RESET_DEVICE         = 0xC0,
    dSPIN_SOFT_STOP            = 0xB0,
    dSPIN_HARD_STOP            = 0xB8,
    dSPIN_SOFT_HIZ             = 0xA0,
    dSPIN_HARD_HIZ             = 0xA8,
    dSPIN_GET_STATUS           = 0xD0
}dSPIN_COMMANDS;


typedef enum dSPIN_REG_ADDR_CONST
{
    // L6470 register address.
    dSPIN_ABS_POS              = 0x01,
    dSPIN_EL_POS               = 0x02,
    dSPIN_MARK                 = 0x03,
    dSPIN_SPEED                = 0x04,
    dSPIN_ACC                  = 0x05,
    dSPIN_DEC                  = 0x06,
    dSPIN_MAX_SPEED            = 0x07,
    dSPIN_MIN_SPEED            = 0x08d,
    dSPIN_FS_SPD               = 0x15,
    dSPIN_KVAL_HOLD            = 0x09,
    dSPIN_KVAL_RUN             = 0x0A,
    dSPIN_KVAL_ACC             = 0x0B,
    dSPIN_KVAL_DEC             = 0x0C,
    dSPIN_INT_SPD              = 0x0D,
    dSPIN_ST_SLP               = 0x0E,
    dSPIN_FN_SLP_ACC           = 0x0F,
    dSPIN_FN_SLP_DEC           = 0x10,
    dSPIN_K_THERM              = 0x11,
    dSPIN_ADC_OUT              = 0x12,
    dSPIN_OCD_TH               = 0x13,
    dSPIN_STALL_TH             = 0x14,
    dSPIN_STEP_MODE            = 0x16,
    dSPIN_ALARM_EN             = 0x17,
    dSPIN_CONFIG               = 0x18,
    dSPIN_STATUS               = 0x19,

} dSPIN_REG_ADDR_CONST;


typedef enum dSPIN_ERROR_CODE
{
    dSPIN_ERR_BADCMD          = 1,
    dSPIN_ERR_NOEXEC          = 2,
    dSPIN_ERR_UVLO            = 4,
    dSPIN_ERR_OVERC           = 8,
    dSPIN_ERR_THSHTD          = 16,
    dSPIN_ERR_STALLA          = 32,
    dSPIN_ERR_STALLB          = 64,
} dSPIN_ERROR_CODE;


// Utility function: get parameter length, in bytes, associated to a given register or command.
uint8_t getParamLength(uint8_t const& param)
{
    switch (param)
    {
        case dSPIN_ABS_POS: return 3;
        case dSPIN_EL_POS: return 2;
        case dSPIN_MARK: return 3;
        case dSPIN_SPEED: return 3;
        case dSPIN_ACC: return 2;
        case dSPIN_DEC: return 2;
        case dSPIN_MAX_SPEED: return 2;
        case dSPIN_MIN_SPEED: return 2;
        case dSPIN_FS_SPD: return 2;
        case dSPIN_INT_SPD: return 2;
        case dSPIN_CONFIG: return 2;
        case dSPIN_STATUS: return 2;
        default: return 1;
    }
}
