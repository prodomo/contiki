/************************************************************************************************/

/**
 * \brief  It sends a generic modbus query
 * \param  data Data to be sent
 * \param  dataLen Length of data
 * \param  crc If 1, crc is used, else crc equals to 0.
 * \return 0 success, -1 error.
 */
int modbusSendQuery(unsigned char *data, unsigned char dataLen, unsigned char hasCRC);
/************************************************************************************************/

/**
 * \brief  It reads the response from modbus query
 * \param  data Pointer to where data will be written
 * \param  hasCRC If equals to 1, modbus query has requested a packet with CRC, 0 else.
 * \return Length of data field in case of success, -1 in case of error.
 */
int modbusReadResponse(unsigned char *data, unsigned char hasCRC);
/************************************************************************************************/

/**
 * \brief  It builds the modbus query packet
 * \param  modbusQuery Structure that contains the packet data
 * \param  packet String that is filled inside this function
 * \return lenght of packet
 */
int modbusBuildQueryPacket(st_modbusQuery *modbusQuery, unsigned char *queryPacket);
/************************************************************************************************/

/**
 * \brief  It verifies errors + exception errors + crc
 * \param  responsePacket Packet of response
 * \param  byteCount Quantity of data bytes
 * \param  exceptionCode pointer to exceptionCode structure
 * \param  hasCrc Parameter that says if the package has CRC or not
 * \param  -1 error, 0 ok.
 */
int modbusRespErrorsVerify(unsigned char *responsePacket, int byteCount, st_modbusExceptionCode *exceptionCode, unsigned char hasCrc);
/************************************************************************************************/

/**
 * \brief  Function to read the output status of up to 2000 coils.
 * \param  modbusQuery Structure with modbus query data
 * \param  coilStatus status of the coils read
 * \param  exceptionCode Pointer to exceptionCode structure
 * \param  hasCrc Parameter that says if the package has CRC or not
 * \return 0 success, -1 error.
 */
int modbusReadCoilStatus(st_modbusQuery *modbusQuery, st_modbusIOGeneric *coilStatus, st_modbusExceptionCode *exceptionCode, unsigned char hasCrc, unsigned int registerModBus);
/************************************************************************************************/
