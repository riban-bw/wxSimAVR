#pragma once

#include <wx/thread.h>
#include <wx/event.h>
#include <wx/filename.h>
extern "C" {
#include "uart_pty.h"
#include "sim_vcd_file.h"
#include <sim_avr_types.h>
}

DECLARE_EVENT_TYPE(wxEVT_AVR_STATUS, wxCommandEvent); //!< Event when avr running status changes. Status stored in Int: AVR_STATUS_STOPPED | AVR_STATUS_RUNNING | AVR_STATUS_PAUSED | AVR_STATUS_CRASHED
DECLARE_EVENT_TYPE(wxEVT_AVR_WRITE, wxCommandEvent); //!< Event when AVR writes to a pin. Pin stored in Int. Value stored in ExtraLong.

struct avr_flash {
	char avr_flash_path[1024];
	int avr_flash_fd;
};

/** CPU Status */
enum
{
    AVR_STATUS_LIMBO = 0,
    AVR_STATUS_STOPPED,
    AVR_STATUS_RUNNING,
    AVR_STATUS_SLEEPING,
    AVR_STATUS_STEP,
    AVR_STATUS_STEPDONE,
    AVR_STATUS_DONE,
    AVR_STATUS_CRASHED,

    AVR_STATUS_PAUSED
};


/** @brief  Class implements an AVR simulator in own thread */
class wxAvr : public wxThread
{
    public:
        /** @brief  Constructs a wxAvr instance
        *   @param  pHandler wxWidgets event handler to recieve events
        *   @param  sType Name of the AVR MCU type to simulate
        *   @param  lFrequency Frequency to run MCU simulation
        */
        wxAvr(wxEvtHandler* pHandler, wxString sType, long lFrequency);

        /** @brief  Called when thread exits
        */
        void OnExit();

        /** @brief  Initializes a new AVR instance. Will call the IO registers init(), and then reset()
        */
        void Init();

        /** @brief  Start AVR MCU
        */
        void Start();

        /** @brief  Stop and reset AVR MPU
        */
        void Stop();

        /** @brief  Pause thread
        *   @return <i>wxThreadError</i> Error number (see wxThread::Pause)
        */
        wxThreadError Pause();

        /** @brief  Resume thread
        *   @return <i>wxThreadError</i> Error number (see wxThread::Resume)
        */
        wxThreadError Resume();

        /** @brief  Resets the AVR, and the IO modules
        */
        void Reset();

        /** @brief  Run one cycle of the AVR, sleep if necessary
        *   @return <i>int</i> Simulator state.
        */
        int Step();

        /** :brief  Set the MCR frequency
        *   @param  lFrequency Frequency
        */
        void SetFrequency(long lFrequency);

        /** @brief  Load code to the "flash"
        *   @param  pCode Pointer to the code
        *   @param  nSize number of bytes to load
        *   @param  nAddress Position in AVR memory to load code
        */
        void LoadCode(unsigned char* pCode, unsigned int nSize, unsigned int nAddress);

        /** @brief  Load firmware to the "flash"
        *   @param  sFirmware Firmware filename
        *   @return <i>bool</i> True on success
        */
        bool LoadFirmware(wxString sFirmware);

        /** @brief  Loads an Intel hex file
        *   @param  sFilename Intel hex filename
        *   @return <i>unsigned int</i> Size of data
        */
        unsigned int LoadHex(wxString sFilename);

        /** @brief  Enable GDB debugger
        *   @param  nPort GDB port number
        */
        void StartDebugger(unsigned int nPort);

        /** @brief  Disable GDB debugger
        */
        void StopDebugger();

        /** @brief  Starts the UART
        *   @param  cPort Port number, e.g. '0'
        */
        void StartUart(char cPort);

        /** @brief  Stops the UART
        */
        void StopUart();

        /** @brief  Initialise VCD logger
        *   @param  vcdFilename Filename to export VCD data to
        *   @param  nPeriod Quantity of microseconds betweeen file flushes
        */
        void InitVcd(const wxFileName& Filename, unsigned int nPeriod);

        /** @brief  Adds a trace signal to be monitored to VCD logger
        *   @param  nPort Index of I/O port to monitor
        *   @param  sName Name to display for signal
        *   @note   Must add signals before starting logging
        */
        void AddSignal(unsigned int nPort, const wxString &sName);

        /** @brief  Register for event when AVR writes to pin
        *   @param  nPin AVR pin
        */
        void RegisterWrite(unsigned int nPin);

        /** @brief  Start VCD logging
        *   @note   Call InitVcd first to create logger
        */
        void StartVcd();

        /** @brief  Stop VCD logging
        */
        void StopVcd();

        /** @brief  Get the current MCU frequency
        *   @return <i>unsigned long</i> Frequency in Hz
        */
        unsigned long GetFrequency();

        /** @brief  Get the current MCU type
        *   @return <i>wxString</i> Name of MCU type
        */
        wxString GetMcuType();

        /** @brief  Get the running status of the MCU
        *   @return <i>unsigned int</i> Status
        */
        unsigned int GetStatus();

        /** @brief  Get list of names (aliases) for MCU type
        *   @param  nIndex Index of MCU type
        *   @return <i>wxArrayString</i> Array of configured MCU types. Empty if invalid index
        */
        static wxArrayString GetMcuNames(unsigned int nIndex);

        /** @brief  Get the size of flash memory in bytes
        *   @return Flash size
        */
        unsigned int GetFlashSize();

        /** @brief  Get the Size of the SRAM in bytes
        *   @return SRAM size
        */
        unsigned int GetSramSize();

        /** @brief  Get the size of EEPROM in bytes
        *   @return EEPROM size
        */
        unsigned int GetEepromSize();

        /** @brief  Get the Vcc
        *   @return Vcc in millivots
        */
        unsigned int GetVcc();

        /** @brief  Get the analogue Vcc
        *   @return aVcc in millivots
        */
        unsigned int GetAvcc();

        /** @brief  Get the analogue reference
        *   @return Reference in millivots
        */
        unsigned int GetaRef();

    protected:
        /** @brief  Thread entry point
        */
        void* Entry();

        wxEvtHandler* m_pEventHandler; //!< Pointer to parent event handler

    private:

        /** @brief  Destroys VCD logger
        *   @note   Stops logging and closes file
        */
        void DestroyVcd();

        /** @brief  Callback function to handle IO write
        */
        static void HandleAvrWrite(avr_t* avr, avr_io_addr_t addr, uint8_t v, void * param);

        /** @brief  Send current MCU state event to parent
        */
        void SendStateEvent();

        avr_t * m_pAvr; //!< Pointer to the AVR simulator
        uart_pty_t* m_pUart; //!< Pointer to UART
        int m_nDo_button_press;
        wxString m_sType; //!< AVR chip type
        unsigned long m_lFrequency; //!< AVR chip frequency
        avr_vcd_t* m_pVcd; //!< Pointer to Value Change Dump logger
        uint8_t	m_nPin_state;
        avr_flash m_flash; //!< Flash memory
        bool m_bCrashed; //!< True if crashed
        wxMutex m_mutex; //!< Mutex protects pointer to avr from multi-thread access
        int m_nState; //!< Current running state of AVR
};

