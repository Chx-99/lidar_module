/*******************************************************************************
****FilePath: /Photonix/src/plc_module/include/plc_module/comm_interface.h
****Author: mwt 911608720@qq.com
****Date: 2025-04-15 09:00:47
****Description:
****Copyright: 2025 by mwt, All Rights Reserved.
********************************************************************************/
#ifndef COMM_INTERFACE_HPP
#define COMM_INTERFACE_HPP

#include <string>
#include <vector>

class CommInterface
{
public:
	virtual ~CommInterface() = default;

	virtual void connect(int slave_id) = 0;
	virtual void disconnect() = 0;
	virtual bool readHoldingRegisters(int address, int count,
									  std::vector<uint16_t> &out) = 0;
	virtual bool readInputRegisters(int address, int count,
									std::vector<uint16_t> &out) = 0;

	virtual bool writeRegisters(int address,
								const std::vector<uint16_t> &data) = 0;

	virtual bool writeCoil(int address, bool value) = 0;

	virtual std::string info() const = 0;

	virtual void setSlaveId(int) const = 0;
};
#endif // COMM_INTERFACE_HPP
