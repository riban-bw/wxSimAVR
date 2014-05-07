#include "wxsimavr.h"
#include <wx/log.h>

extern "C" {
#include "sim_avr.h"
#include "avr_ioport.h"
#include "sim_elf.h"
#include "sim_hex.h"
#include "sim_gdb.h"
#include "avr_twi.h"
#include "sim_core_decl.h" //!@todo must ensure this is same as compiled simavr library uses - defines available avr cores
}

DEFINE_EVENT_TYPE(wxEVT_AVR_STATUS); //event updates host with current avr status
DEFINE_EVENT_TYPE(wxEVT_AVR_WRITE); //event informs host of write to avr pin

wxAvr::wxAvr(wxEvtHandler* pHandler, wxString sType, long lFrequency) :
    wxThread(wxTHREAD_DETACHED),
    m_pEventHandler(pHandler),
    m_pAvr(NULL),
    m_pUart(NULL),
    m_sType(sType),
    m_lFrequency(lFrequency),
    m_pVcd(NULL),
    m_bCrashed(false),
    m_nState(AVR_STATUS_LIMBO)
{
    if(!m_pAvr)
        m_pAvr = avr_make_mcu_by_name(m_sType.mb_str());
    if(!m_pAvr)
    {   //Unable to create avr core
        //!@todo send error message (event)
        return;
    }
    avr_init(m_pAvr);
	m_pAvr->frequency = m_lFrequency; //do this after init because init sets to default 1000000
}

void wxAvr::OnExit()
{
    wxLogDebug(_("wxAvr::OnExit"));

//    close(m_flash.avr_flash_fd);
    DestroyVcd();
    if(m_pUart)
        StopUart();
    m_mutex.Lock(); //About to change avr pointer so lock the thread
    if(m_pAvr)
        avr_terminate(m_pAvr);
    m_pAvr = NULL;
    m_mutex.Unlock();
}

void* wxAvr::Entry()
{
    int nState = AVR_STATUS_RUNNING;
    if(!m_pAvr)
        return NULL; //Need to call Create() first
    m_nState = AVR_STATUS_RUNNING;
    SendStateEvent();
    while(!TestDestroy())
    {
        if(!m_pAvr)
            return NULL; //!@todo Why are we checking this here?
        if(m_nState == AVR_STATUS_RUNNING)
            nState = avr_run(m_pAvr); //Run one cycle of AVR
        else
            nState = m_nState;
		if(nState != m_nState)
        {
            m_nState = nState;
            if(nState == cpu_Done || nState == cpu_Crashed)
                wxLogDebug(_("AVR finished with state %d"), nState);
            SendStateEvent();
        }
    }
    return NULL;
}

void wxAvr::Init()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        avr_init(m_pAvr);
}

void wxAvr::Start()
{
    wxMutexLocker locker(m_mutex);
    if(IsRunning())
        return;
    if(!m_pAvr)
        return;
//	StartUart();
    if(IsPaused())
        Resume();
    else
        Run();
}

void wxAvr::Stop()
{
    wxMutexLocker locker(m_mutex);
    if(IsRunning())
        wxThread::Pause();
    if(m_pAvr)
        avr_reset(m_pAvr);
    m_nState = AVR_STATUS_STOPPED;
    SendStateEvent();
}

wxThreadError wxAvr::Pause()
{
    wxMutexLocker locker(m_mutex);
    if(!IsRunning())
        return wxTHREAD_NOT_RUNNING;
    wxThreadError nReturn = wxThread::Pause();
    m_nState = AVR_STATUS_PAUSED;
    SendStateEvent();
    return nReturn;
}

wxThreadError wxAvr::Resume()
{
    wxMutexLocker locker(m_mutex);
    if(!IsPaused())
        return wxTHREAD_MISC_ERROR;
    m_nState = AVR_STATUS_RUNNING;
    SendStateEvent();
    return wxThread::Resume();
}

void wxAvr::Reset()
{
    //!@todo With Arduino bootloader, this seems to resets to start of Arduino program, not bootloader - I think this is because bootloader is being overwritten
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return;
    bool bRunning = IsRunning();
    if(bRunning)
        wxThread::Pause();
    avr_reset(m_pAvr);
    if(bRunning)
        wxThread::Resume();
}

int wxAvr::Step()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return avr_run(m_pAvr);
    return cpu_Limbo;
}

void wxAvr::SetFrequency(long lFrequency)
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        m_pAvr->frequency = lFrequency;
}

void wxAvr::LoadCode(unsigned char* pCode, unsigned int nSize, unsigned int nAddress)
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        avr_loadcode(m_pAvr, pCode, nSize, nAddress);
}

bool wxAvr::LoadFirmware(wxString sFirmware)
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return false;
    elf_firmware_t firmware;
    if(elf_read_firmware(sFirmware.mbc_str(), &firmware))
        return false;
    avr_load_firmware(m_pAvr, &firmware);
    return true;
}

unsigned int wxAvr::LoadHex(wxString sFilename)
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return 0;
    if(!wxFileExists(sFilename))
        return 0;
    unsigned int nSize, nBase;
    uint8_t * pBuffer = read_ihex_file(sFilename.mbc_str(), &nSize, &nBase);
    if(!pBuffer)
        return 0;
    if(nBase + nSize > m_pAvr->flashend) //!@todo this may be checked in read_ihex_file
    {
        free(pBuffer);
        wxLogMessage("wxAvr::Loadhex failed: Data larger than flash");
        return 0;
    }
    memcpy(m_pAvr->flash + nBase, pBuffer, nSize);
    free(pBuffer);
	m_pAvr->pc = nBase;
    m_pAvr->codeend = m_pAvr->flashend;
    return nBase + nSize;
}

void wxAvr::StartDebugger(unsigned int nPort)
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return;
    if(m_pAvr->gdb_port)
        return; //Already initiated
    m_pAvr->gdb_port = nPort;
    m_pAvr->state = cpu_Stopped ;
    avr_gdb_init(m_pAvr) ;
}

void wxAvr::StopDebugger()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return;
    avr_deinit_gdb(m_pAvr);
    m_pAvr->gdb_port = 0;
}

void wxAvr::StartUart(char cPort)
{
    wxMutexLocker locker(m_mutex);
    if(m_pUart)
        return; //Already created
    m_pUart = new uart_pty_t;
	uart_pty_init(m_pAvr, m_pUart);
	uart_pty_connect(m_pUart, cPort);
	return;
}

void wxAvr::StopUart()
{
    wxMutexLocker locker(m_mutex);
    if(m_pUart)
        uart_pty_stop(m_pUart);
    m_pUart = NULL;
}

void wxAvr::InitVcd(const wxFileName& vcdFilename, unsigned int nPeriod)
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr || m_pVcd)
        return;
    m_pVcd = new avr_vcd_t;
    avr_vcd_init(m_pAvr, vcdFilename.GetFullPath().mbc_str(), m_pVcd, nPeriod);
}

void wxAvr::DestroyVcd()
{
    //Don't use mutex on private functions else we may find ourselves deadlocked - ensure these are always called from worker thread, i.e. entry or destroy
    if(!m_pVcd)
        return;
    StopVcd();
    avr_vcd_close(m_pVcd);
    delete m_pVcd;
    m_pVcd = NULL;
}

void wxAvr::AddSignal(unsigned int nPort, const wxString &sName)
{
    wxMutexLocker locker(m_mutex);
    if(!m_pVcd || !m_pAvr)
        return;
    avr_vcd_add_signal(m_pVcd, avr_io_getirq(m_pAvr, AVR_IOCTL_IOPORT_GETIRQ(nPort), TWI_IRQ_OUTPUT), 8, sName.mbc_str());
}

void wxAvr::StartVcd()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pVcd)
        return;
    avr_vcd_start(m_pVcd);
}

void wxAvr::StopVcd()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pVcd)
        return;
    avr_vcd_stop(m_pVcd);
}

void wxAvr::RegisterWrite(unsigned int nPin)
{
    //!@todo Check if registration already exists (might be done by avrsim already - check
    avr_register_io_write(m_pAvr, (avr_io_addr_t)nPin, HandleAvrWrite, this);
}

//static void avr_flash_write(avr_t * avr, avr_io_addr_t addr, uint8_t v, void * param)
void wxAvr::HandleAvrWrite(avr_t* avr, avr_io_addr_t addr, uint8_t v, void * param)
{
    wxAvr* pThis = static_cast<wxAvr*>(param);
    if(!pThis)
        return;
    wxCommandEvent event(wxEVT_AVR_WRITE);
    event.SetInt(addr);
    event.SetExtraLong((long) param);
    wxPostEvent(pThis->m_pEventHandler, event);
}

unsigned long wxAvr::GetFrequency()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return 0;
    return m_pAvr->frequency;
}

wxString wxAvr::GetMcuType()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return wxEmptyString;
    return wxString::Format(_("%s"), m_pAvr->mmcu);
}

void wxAvr::SendStateEvent()
{
    wxCommandEvent event(wxEVT_AVR_STATUS, GetId());
    event.SetInt(m_nState);
    wxPostEvent(m_pEventHandler, event);
}

unsigned int wxAvr::GetStatus()
{
    wxMutexLocker locker(m_mutex);
    return m_nState;
}

wxArrayString wxAvr::GetMcuNames(unsigned int nIndex)
{
	wxArrayString asNames;
	if(avr_kind[nIndex]) //!@todo Is it safe to access array without bounds check? It seems to work and is used by simavr!
    {
        for(int nAlias = 0; avr_kind[nIndex]->names[nAlias]; nAlias++)
            asNames.Add(avr_kind[nIndex]->names[nAlias]);
	}
	return asNames;
}

unsigned int wxAvr::GetFlashSize()
{
    wxMutexLocker locker(m_mutex);
    if(!m_pAvr)
        return 0;
    return m_pAvr->flashend + 1;
}

unsigned int wxAvr::GetSramSize()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return m_pAvr->ramend + 1;
    return 0;
}

unsigned int wxAvr::GetEepromSize()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return m_pAvr->e2end + 1;
    return 0;
}

unsigned int wxAvr::GetVcc()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return m_pAvr->vcc;
    return 0;
}

unsigned int wxAvr::GetAvcc()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return m_pAvr->avcc;
    return 0;
}

unsigned int wxAvr::GetaRef()
{
    wxMutexLocker locker(m_mutex);
    if(m_pAvr)
        return m_pAvr->aref;
    return 0;
}
