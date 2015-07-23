#ifndef POLARIS_SENSOR_H
#define POLARIS_SENSOR_H
#include <stdint.h>
#include <vector>
#include <string>
#include <map>
// Included in ROS but available at : http://wjwwood.io/serial/doc/1.1.0/index.html
#include <serial/serial.h>
namespace polaris{
class TransformationDataBX
{
public:
    float q0,qx,qy,qz,tx,ty,tz,error;

    /**
     * @brief Port status :
     * Bit | Status
     * --- | ------
     * 0 | Occupied
     * 1 | GPIO 1
     * 2 | GPIO 2
     * 3 | GPIO 3
     * 4 | Initialized
     * 5 | Enabled
     * 6 | Out of volume
     * 7 | Partially out of volume
     */

    uint32_t portStatus;
    /**
     * @brief Handle status :
     * Value | Status
     * ----- | ------
     * 01 | Valid
     * 02 | Missing
     * 04 | Disabled
     */
    uint8_t handleStatus;

    /**
     * @brief Frame number
     */
    uint32_t number;
};

class TransformationDataTX
{
public:
    float q0,qx,qy,qz,tx,ty,tz,error;
    std::string status;
    std::string number;
};

/**
 * @brief The QPolaris class is a wrapper used to communicate with a Polaris camera.
 */
class Polaris
{

public:
    Polaris();
    /**
     * @brief Polaris Standard constructor.
     */
    Polaris(const std::string port,const std::vector<std::string> roms);
    /**
     * @brief Standard destructor. Closes the com port if it is still open.
     */
    ~Polaris();

    /**
     * @brief Opens the port
     * @param comPort COM Port the Polaris is using.
     * @param baudrate Baud rate of the COM port, can be BAUD9600 or BAUD115200.
     * @return true in case of success.
     */
    bool openPort(const std::string& portname, uint32_t baudrate);

    /**
     * @brief Closes the COM port if it is open.
     * @return true in case of success.
     */
    void closePort();

    /**
     * @brief Changes the Polaris' baud rate. The port must be already opened by openPort().
     * @param baudrate Baud rate of the Polaris, can be BAUD9600 or BAUD115200.
     */
    bool setPolarisBaudRateTo(unsigned long baudrate);

    /**
     * @brief Initializes the Polaris.
     */
    void init();

    /**
     * @brief Clears previous port handles; Must be called after init().
     */
    void clearPortHandles();

    /**
     * @brief Request a handle from the Polaris.
     * @param handle Variable to fill with the handle id returned by the Polaris.
     */
    std::string requestPortHandle();

    /**
     * @brief Attaches a ROM file to a specific handle.
     * @param handle Handle given by requestPortHandle().
     * @param filename Path to the rom file.
     */
    bool loadROM(const std::string&handle, std::string filename);

    /**
     * @brief Initializes a port handle. You must first load a ROM file with loadROM()
     * @param handle Handle given by requestPortHandle().
     */
    void initPortHandle(const std::string& handle);

    /**
     * @brief Enables a handle.
     * @param handle Handle given by requestPortHandle().
     * @param priority Can be either 'S'(tatic) or 'D'(ynamic). Dynamic has a higher priority.
     */
    void enablePortHandle(const std::string&handle, char priority);

    /**
     * @brief Starts the tracking.
     */
    void startTracking();

    /**
     * @brief Stops the tracking.
     */
    void stopTracking();

    /**
     * @brief Reads available data as TX mode (slower than BX). Don't use this.
     * @param systemStatus System status.
     * @return A map between handles and transformation datas.
     */
    void readDataTX(std::string &systemStatus,std::map<int,TransformationDataTX>& map);

    /**
     * @brief readDataBX Reads available data as BX mode (binary).
     * @param systemStatus System status :
     * Bit | Status
     * --- | ------
     * 0 | System communication synchronization error
     * 1 | Too much external IR
     * 2 | System CRC error
     * 3 | Recoverable system processing exception
     * 6 | Some port handle has become occupied
     * 7 | Some port handle had become unoccupied
     * @param map A map between handles (as int) and transformation datas.
     * @see byteArray2int()
     */
    void readDataBX(uint16_t &systemStatus, std::map<int, TransformationDataBX>& map );

    /**
     * @brief Reads information available for a specific port.
     * @param handle Handle given by requestPortHandle().
     * @return
     */
    std::string readPortInfo(const std::string& handle);

    /**
     * @brief Reads the serial number of a specific tool.
     * @param handle Handle given by requestPortHandle().
     * @return 8 hexadecimal characters
     */
    std::string readToolSerialNumber(const std::string& handle);

    /**
     * @brief Reads the port number of the tool.
     * @param handle Handle given by requestPortHandle().
     * @return 2 ASCII characters : 01-12 Wired tool, 0A-0I Wireless tool
     */
    std::string readToolPortNumber(const std::string& handle);

    /**
     * @brief Transforms an ascii char to an integer (ie: '8' -> 8, 'B' -> '11').
     * @param c Ascii char. Must be uppercase.
     * @return Integer value of c.
     */
    static int ascii2int(char c);

    /**
     * @brief Transforms a std::string to an integer given a base.
     * @param buf std::string to transform.
     * @param base Base to use (10 for decimal, 16 for hex, etc..).
     * @return The converted value.
     */
    static int PortHandle2int(const std::string& handle, int base = 16);
    /**
     * @brief Removes the '\r' in a string.
     * @param str std::string to transform.
     * @param c the char to remove.
     * @return Nothing.
     */
    static void removeChar(std::string& str,const char c);
    const unsigned int getNumberOfTargets()const{return number_of_targets_;}
private:
    unsigned int number_of_targets_;
    serial::Serial m_port;

    std::string readUntilCR(uint32_t timeout = 500);
    std::string readUntilBXAnswerComplete();

    /**
     * @brief Analyzes a Polaris answer.
     * @param buf Buffer.
     * @param sz Size of buf.
     * @return 0 if answer was OKAY, >0 if answer was an error, <0 if answer couldn't be interpreted
     */
    static int checkAnswer(const std::string &answer);
};
}
#endif
