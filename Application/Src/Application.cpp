//==============================================================================
/**
 * @file Application.cpp
 * @title Communication Module CTB-02 - Placar Tenis PLT-01
 * @author Joao Nilo Rodrigues  -  nilo@pobox.com
 */
//------------------------------------------------------------------------------
// NOTA: em caso de necessidade de mais espaço na memória flash remover a
// variável BatteryVoltage (float) passando o cálculo final para o Android.
//------------------------------------------------------------------------------
// 04/07/2023: Detecção do carregador de bateria não funciona quando este já
// está presente ao ligar esse módulo.
// Parâmetro "ScoreStatus" passa ser enviado aos displays com o novo flag
// STATUS_FLAGS_BATTERYLOW para que os displays desliguem seus drivers dos servos
// quando a beteria estiver descarregada.
// 21/08/2023: clock do sistema alterado para 16MHz para reduzir consumo. [1.0.0.3]
// 11/12/2023: sinalização da conexão pelo LED. Limpeza de código comentado [1.0.0.4]
//------------------------------------------------------------------------------
#include "Application.h"
#include "System.h"
//#define TEST_1

//------------------------------------------------------------------------------
// NOTE: product ID, firmware version and piblishing date
//------------------------------------------------------------------------------
const uint32_t productID 			= 0x00000002;
const uint8_t firmwareVersion[4] 	= {1,0,0,4};
const uint8_t publishingDate[4]  	= {11,12,20,23};

NLed* Led_Heartbeat;
NLed* FullyCharged;
NTinyOutput* PowerBLE;
NTinyOutput* PowerAmp;
NSwitch* ChargerPluged;
NInput* Connected;

NTimer* Timer1;
NAdc* Analogs;
NFilter* Vbat;

// Bus protocol components
NSerial* BusPort;
NTinyOutput* BusPort_DE;
NTinyOutput* BusPort_RE;
NDatagram* BUS_OutData;
NDataLink* BUS_Link;
NSerialProtocol* BUS_Interpret;
NSerialCommand* busGetStatus;
NSerialCommand* busSetData;

// Bluetooth protocol components
NSerial* BlePort;
NDatagram* BLE_OutData;
NDataLink* BLE_Link;
NSerialProtocol* BLE_Interpret;
NSerialCommand* bleGetVersion;
NSerialCommand* bleGetStatus;
NSerialCommand* bleSetData;

//------------------------------------------------------------------------------
// data section
#define SCORE_PARAMS_SIZE		14
#define PARAMS_PLAY1_TENS		0
#define PARAMS_PLAY1_UNITS		1
#define PARAMS_PLAY1_SET1		2
#define PARAMS_PLAY1_SET2		3
#define PARAMS_PLAY1_SET3		4
#define PARAMS_PLAY2_TENS		5
#define PARAMS_PLAY2_UNITS		6
#define PARAMS_PLAY2_SET1		7
#define PARAMS_PLAY2_SET2		8
#define PARAMS_PLAY2_SET3		9
#define PARAMS_FLAGS			10
#define PARAMS_SECONDS			11
#define PARAMS_MINUTES			12
#define PARAMS_HOURS			13
uint8_t ScoreParams[SCORE_PARAMS_SIZE];

//------------------------------------------------------------------------------
#define PARAMS_FLAGS_SERV_PLAY1		((uint8_t) 0x01)
#define PARAMS_FLAGS_SERV_PLAY2		((uint8_t) 0x02)
#define PARAMS_FLAGS_SERV_MASK		((uint8_t) 0x03)
#define PARAMS_FLAGS_CONNECTED		((uint8_t) 0x40)
#define PARAMS_FLAGS_CALIBRATING	((uint8_t) 0x80)

//------------------------------------------------------------------------------
#define PARAMS_DISPLAY_BLANK		((uint8_t) 0x10)

//------------------------------------------------------------------------------
uint16_t ScoreStatus = 0;
uint8_t Settings = 0;
float BatteryVoltage;
uint8_t connection = 0;

//------------------------------------------------------------------------------
bool new_data = false;
uint8_t fsm_counter = BUS_NODES;
uint8_t node_faults[BUS_NODES];
#define FAULTS_ALLOWED 	10

//------------------------------------------------------------------------------
const uint8_t NodeAddresses[BUS_NODES] = {
		PLAY1_TENS, PLAY1_UNITS, PLAY1_SET1, PLAY1_SET2, PLAY1_SET3,
		PLAY2_TENS, PLAY2_UNITS, PLAY2_SET1, PLAY2_SET2, PLAY2_SET3
};

//------------------------------------------------------------------------------
void Timer1_OnTimer();
void Connected_OnChange();
void Analogs_OnData(uint16_t);

void BusPort_OnPacket(uint8_t* data, uint8_t size);
void BusPort_OnEnterTransmission();
void BusPort_OnLeaveTransmission();
void BusLink_OnPacketToSend(uint8_t*, uint8_t);
void BusLink_OnReload(NDatagram*);
void BusLink_OnSilence();
void busGetVersion_OnProcess(NDatagram*);
void busGetStatus_OnProcess(NDatagram*);
void busSetData_OnProcess(NDatagram*);

void BlePort_OnPacket(uint8_t* data, uint8_t size);
void BleLink_OnPacketToSend(uint8_t*, uint8_t);
void bleGetVersion_OnProcess(NDatagram*);
void bleGetStatus_OnProcess(NDatagram*);
void bleSetData_OnProcess(NDatagram*);

void AddressResolution();
uint8_t GetNodeIndex(uint8_t);

//------------------------------------------------------------------------------
void ApplicationCreate(){

 	Led_Heartbeat = new NLed(LED);
 	Led_Heartbeat->Interval = 500;
	Led_Heartbeat->Status = ldBlinking;

    Analogs = new NAdc(ADC1);
    Analogs->AddChannel(adCH0);
    Analogs->Mode = adContinuous1;
    Analogs->OnData = Analogs_OnData;
    Analogs->Start(250);

    Vbat = new NFilter(5);

    //--------------------------------------------------------------------------
    // Bluetooth communication components
    BlePort = new NSerial(USART2, seStandard); // Tx: PA2 Rx: PA3
    BlePort->BaudRate = 115200;
    BlePort->OnPacket = BlePort_OnPacket;
    BlePort->Open();

    BLE_Link = new NDataLink();
    BLE_Link->OnPacketToSend = BleLink_OnPacketToSend;
    BLE_Link->LocalAddress = PROSA_ADDR_DEVICE1;
    BLE_Link->ServiceAddress = PROSA_ADDR_SERVICE;
    BLE_Link->BroadcastAddress = PROSA_ADDR_BROADCAST;

    BLE_Interpret = new NSerialProtocol(BLE_Link);

    bleGetVersion = new NSerialCommand(BLE_Interpret);
    bleGetVersion->ID = PROSA_CMD_VERSION;
    bleGetVersion->OnProcess = bleGetVersion_OnProcess;

    bleGetStatus = new NSerialCommand(BLE_Interpret);
    bleGetStatus->ID = PROSA_CMD_GETSTATUS;
    bleGetStatus->OnProcess = bleGetStatus_OnProcess;

    bleSetData = new NSerialCommand(BLE_Interpret);
    bleSetData->ID = PROSA_CMD_SETDATA;
    bleSetData->OnProcess = bleSetData_OnProcess;

	BLE_OutData = new NDatagram();
	BLE_OutData->Source = BLE_Link->LocalAddress;
	BLE_OutData->Destination = BLE_Link->BroadcastAddress;

    //--------------------------------------------------------------------------
    // Bus communication components
    BusPort = new NSerial(BUS_PORT); 			// Tx: PA9 Rx: PA10  DE: PA12  RE: PA11
    BusPort->BaudRate = 115200;
    BusPort->OnPacket = BusPort_OnPacket;
    BusPort->OnEnterTransmission = BusPort_OnEnterTransmission;
    BusPort->OnLeaveTransmission = BusPort_OnLeaveTransmission;
    BusPort->Open();

    BUS_OutData = new NDatagram();
    BUS_OutData->Destination = PROSA_ADDR_BROADCAST;
    BUS_OutData->Source = PROSA_ADDR_IHM1;
    BUS_OutData->Command = PROSA_CMD_VERSION;

    BUS_Link = new NDataLink();
    BUS_Link->TimeReload = 100;
    BUS_Link->TimeDispatch = 40;
    BUS_Link->Timeout = 15;
    BUS_Link->BusPrivilege = dlSlave;
    BUS_Link->LocalAddress = PROSA_ADDR_DEVICE1;
    BUS_Link->ServiceAddress = PROSA_ADDR_SERVICE;
    BUS_Link->BroadcastAddress = PROSA_ADDR_BROADCAST;
    BUS_Link->OnPacketToSend = BusLink_OnPacketToSend;
    BUS_Link->OnReload = BusLink_OnReload;
    BUS_Link->OnSilence = BusLink_OnSilence;

    BUS_Interpret = new NSerialProtocol(BUS_Link);

    busGetStatus = new NSerialCommand(BUS_Interpret);
    busGetStatus->ID = PROSA_CMD_GETSTATUS;
    busGetStatus->OnProcess = busGetStatus_OnProcess;

    BusPort_DE = new NTinyOutput(USART1_RTS);
    BusPort_RE = new NTinyOutput(USART1_CTS);

    //--------------------------------------------------------------------------
    Timer1 = new NTimer();
    Timer1->OnTimer = Timer1_OnTimer;
    Timer1->Start(5000);

    PowerBLE = new NTinyOutput(CONN);
    PowerBLE->Level = toHigh;

    PowerAmp = new NTinyOutput(AMP);
    PowerAmp->Level = toHigh;

    FullyCharged = new NLed(SPK);
    FullyCharged->Interval = 750;
    FullyCharged->Status = ldOff;

    ChargerPluged = new NSwitch(CHG);
    ChargerPluged->Bias = inPullUp;
    ChargerPluged->Mode = swSwitch;

    //TODO: checar porque "trava" com a classe NSwitch
    Connected =  new NInput(CONN);
    Connected->Inverted = false;
    Connected->Bias = inPullDown;
    Connected->OnLevelChange = Connected_OnChange;

    for(int c=0; c<SCORE_PARAMS_SIZE; c++){ ScoreParams[c] = PARAMS_DISPLAY_BLANK;}

    BUS_Link->Open();
    BLE_Link->Open();
}

//------------------------------------------------------------------------------
void Timer1_OnTimer(){

	if(ScoreStatus & STATUS_FLAGS_BATTERYLOW){
		// block scoreboard if battery is too low
		Timer1->Start(1000);
	} else {
		Timer1->Stop();
		BUS_Link->BusPrivilege = dlMaster;
		ScoreStatus = 0;
		for(int c=0; c< BUS_NODES; c++){ node_faults[c] = 0;}

		///----------------------------------------------------
		/// BLOCO DE TESTE 1
	    #if defined( TEST_1)
		Analogs->Stop();
		ScoreStatus = 0x3FF;
		Settings = 0xA5;
		BatteryVoltage = BatteryLowerThresold;
		busGetStatus->OnProcess = NULL;
		BUS_Link->OnSilence = NULL;
		#endif
		///----------------------------------------------------
	}
}

//------------------------------------------------------------------------------
void Connected_OnChange(){
	if(Connected->Level == inHigh){
		connection = PARAMS_FLAGS_CONNECTED;
		Led_Heartbeat->Interval = 3000;
		Led_Heartbeat->Duty = 2;
	} else {
		connection = 0;
		Led_Heartbeat->Interval = 500;
		Led_Heartbeat->Duty = 50;
	}
}

//------------------------------------------------------------------------------
void BusPort_OnEnterTransmission(){
	BusPort_RE->Level = toHigh; BusPort_DE->Level = toHigh;
}

//------------------------------------------------------------------------------
void BusPort_OnLeaveTransmission(){
	BusPort_DE->Level = toLow; BusPort_RE->Level = toLow;
}


//------------------------------------------------------------------------------
void BusPort_OnPacket(uint8_t* data, uint8_t size){
	BUS_Link->ProcessPacket(data, size);
}

//------------------------------------------------------------------------------
void BusLink_OnPacketToSend(uint8_t* data, uint8_t size){
	BusPort->Write(data, size);
}

//------------------------------------------------------------------------------
void BusLink_OnReload(NDatagram* iDt){

	if(fsm_counter > BUS_NODES){ fsm_counter = 0;}
	if((new_data) || (fsm_counter == BUS_NODES)){
		new_data = false;
		iDt->Destination = PROSA_ADDR_BROADCAST;
		iDt->Command = PROSA_CMD_SETDATA;
		iDt->Flush();
		iDt->Append(ScoreParams, SCORE_PARAMS_SIZE);
    } else {
		iDt->Destination = NodeAddresses[fsm_counter];
		iDt->Source = BUS_Link->LocalAddress;
		iDt->Command = PROSA_CMD_GETSTATUS;
		iDt->Flush();
		iDt->Append(ScoreStatus);
	}
	fsm_counter++;
}

//------------------------------------------------------------------------------
void BusLink_OnSilence(){
	if(fsm_counter < BUS_NODES){
		if(node_faults[fsm_counter] < FAULTS_ALLOWED){
			node_faults[fsm_counter]++;
		} else if (node_faults[fsm_counter] == FAULTS_ALLOWED){
			node_faults[fsm_counter]++;
			uint16_t NodeMask =  1;
			ScoreStatus &= ~(NodeMask << fsm_counter);
		}
	}
}


//------------------------------------------------------------------------------
void busGetStatus_OnProcess(NDatagram* iDt){
	uint8_t nodeBITE = 0; uint8_t nodeIndex = 11;
	uint16_t nodeFlag = 1; uint8_t nodeAddress = 11;
	if(iDt->Length>1){
		nodeBITE = iDt->Extract();
		nodeAddress = iDt->Extract();

		nodeIndex = GetNodeIndex(nodeAddress);

		if(nodeIndex < BUS_NODES){
			nodeFlag <<= nodeIndex;

			if(nodeBITE == 0){
				ScoreStatus |= nodeFlag;
				node_faults[nodeIndex] = 0;
			} else {
				if(node_faults[nodeIndex] < FAULTS_ALLOWED){
					node_faults[nodeIndex]++;
				} else if (node_faults[nodeIndex] == FAULTS_ALLOWED){
					node_faults[nodeIndex]++;
					ScoreStatus = ScoreStatus & ((uint16_t)(~nodeFlag));
				}
			}
		}
	}
}

//------------------------------------------------------------------------------
// Get the update values from the Control Unit
void busSetData_OnProcess(NDatagram* iDt){}

//------------------------------------------------------------------------------
// BLUETOOTH
//------------------------------------------------------------------------------
void BlePort_OnPacket(uint8_t* data, uint8_t size){
	BLE_Link->ProcessPacket(data, size);
}

//------------------------------------------------------------------------------
void BleLink_OnPacketToSend(uint8_t* data, uint8_t size){
	BlePort->Write(data, size);
}

//------------------------------------------------------------------------------
void bleGetVersion_OnProcess(NDatagram* iDt){
	iDt->SwapAddresses(); iDt->Size = 0;
	iDt->Append(*(uint32_t*) &productID);
	iDt->Append(*(uint32_t*) publishingDate);
	iDt->Append(*(uint32_t*) firmwareVersion);
	iDt->UpdateCrc();
}

//------------------------------------------------------------------------------
void bleGetStatus_OnProcess(NDatagram* iDt){
	iDt->SwapAddresses(); iDt->Flush();
	iDt->Append(ScoreStatus);
	iDt->Append(BatteryVoltage);
	iDt->Append(Settings);
}

//------------------------------------------------------------------------------
// Get the update values from the Control Unit
void bleSetData_OnProcess(NDatagram* iDt){
	//uint8_t result = false;

	if(iDt->Length >= SCORE_PARAMS_SIZE){
		iDt->Extract(ScoreParams, SCORE_PARAMS_SIZE);
		ScoreParams[PARAMS_FLAGS] |= connection;
		new_data = true;
	}

	iDt->SwapAddresses();
 	iDt->Flush();
	iDt->Append(ScoreStatus);
	iDt->Append(BatteryVoltage);
	iDt->Append(Settings);
}

//------------------------------------------------------------------------------
// Vref = 3V3; ADC = 12bit; LSB = 805uV
// Factor = 22K/68K+22K = 0.2444
// Vmax = 2.2 / 0.2444 = 9,0V
// Gain:   805uV  / 0.2444 = 0,0033
void Analogs_OnData(uint16_t new_sample){
	BatteryVoltage = BatteryFactor * Vbat->Filter(new_sample);
	ScoreStatus &= STATUS_MASK_BATTERYLOW;

	if(BatteryVoltage > BatteryUpperThresold){
		if(ScoreStatus & STATUS_FLAGS_CHARGING){
			ScoreStatus |= STATUS_FLAGS_CHARGED;
		}
	} else if(BatteryVoltage < BatteryLowerThresold){
		ScoreStatus &= STATUS_MASK_CHARGED;
		if(BatteryVoltage < BatteryLowerVoltage){
			ScoreStatus |= STATUS_FLAGS_BATTERYLOW;
		}
	}


	// update the "charger presence"
	if(ChargerPluged->Status == swOff){
		ScoreStatus |= STATUS_FLAGS_CHARGING;
	} else {
		ScoreStatus &= STATUS_MASK_CHARGING;
	}

	// update the "battery status" LED
	if((bool)(ScoreStatus & STATUS_FLAGS_CHARGED)){
		FullyCharged->Status = ldBlinking;
	} else {
		FullyCharged->Status = ldOff;
	}
}

//------------------------------------------------------------------------------
void AddressResolution(){

	NInput* Settings0 = new NInput(CFN0);
	NInput* Settings1 = new NInput(CFN1);
	NInput* Settings2 = new NInput(CFN2);

	Settings |= (Settings2->Level << 2);
	Settings |= (Settings1->Level << 1);
	Settings |= (Settings0->Level << 0);

	delete Settings0;
	delete Settings1;
	delete Settings2;
}

//------------------------------------------------------------------------------
uint8_t GetNodeIndex(uint8_t addr){
	uint8_t index = BUS_NODES+1;
	for(int c=0; c < BUS_NODES; c++){
		if(addr == NodeAddresses[c]){ index = c;}
	}
	return(index);
}

//==============================================================================
