
/**
 * \brief  This file contain defines used in modbus protocol functions
 */


 //! Needed to build CRC-16
 #define P_16                                   0xA001


//! CRC_LENGTH
#define MODBUS_CRC_LENGTH                       2

//! query length
#define MODBUS_QUERY_LENGTH                     6

//! function + address length
#define MODBUS_HEADER_LENGTH            2

//{! Exception code defines
#define MODBUS_EXC_ADD                          0x80

#define MODBUS_ILLEGAL_FUNCTION         0x01
#define MODBUS_ILLEGAL_DATA_ADD         0x02
#define MODBUS_ILLEGAL_DATA_VALUE       0x03
#define MODBUS_SLAVE_DEVICE_FAIL        0x04
//@}



//!{Max Length
#define MODBUS_MAX_DATA_LENGTH          252
#define MODBUS_MAX_QUERY_LENGTH         255
#define MODBUS_MAX_DEV_ADDRESSED        246
#define MODBUS_MAX_RESP_LENGTH          1024

#define MODBUS_MAX_NUM_COILS            2000
//@}

//!{ Function codes
#define MODBUS_RD_COIL                  0x01
#define MODBUS_RD_DISCRETE_IN           0x02
#define MODBUS_RD_HOLD_REG              0x03
#define MODBUS_RD_IN_REG                0x04
#define MODBUS_WR_SINGLE_COIL           0x05
#define MODBUS_WR_SINGLE_REG            0x06

#define MODBUS_WR_MULT_COIL             0x0F
#define MODBUS_WR_MULT_REG              0x10
#define MODBUS_REPORT_SLAVE_ID          0x11

#define MODBUS_RD_WR_MULT_REG           0x17
#define MODBUS_RD_FIFO                  0x18

//@}
