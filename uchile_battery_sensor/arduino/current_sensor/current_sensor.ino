#define CURRENT_SENSOR A0
#define VOLTAGE_SENSOR A1

float current = 0;
float voltage = 0;
float transfer_energy = 0;
float SOC = 0;
float Q0 = 5040;//As
float Qt = 0;//As
unsigned long time;


void setup() {
    pinMode(CURRENT_SENSOR, INPUT);
    pinMode(VOLTAGE_SENSOR, INPUT);


    Serial.begin(9600);
}

void loop() {
    time = millis();
    float current_raw = analogRead(CURRENT_SENSOR);
    float voltage_raw = analogRead(VOLTAGE_SENSOR);
    if (current_raw>=550) current = (current_raw-550)*0.0322;
    else current = 0;
    if (voltage_raw>=164) voltage = (voltage_raw-164)*0.025;
    else voltage = 0;
    Qt= Qt+ ((current*(millis()-time))/1000);//As
    transfer_energy = transfer_energy + (voltage*current*(millis()-time)/1000);//Ws
    SOC = (1-(Qt/Q0))*100;//por revisar
    send_data(1, transfer_energy, current, SOC, voltage);
}

void sendFrame(uint8_t frame[], uint8_t sz)
{   
    for (int j=0;j<sz;j++) Serial.write(frame[j]);
}

uint8_t checksum(uint8_t *packet, uint8_t n)
{
    uint32_t transfer_energy = 0;
    for (int j=0;j<n-1;j++) transfer_energy += packet[j];
    return transfer_energy & 0x00FF;
}

void bytesEncode(float number, uint8_t encode_bytes[])
{
    uint16_t int_part = (uint16_t) abs(number);           //[0-65535]
    float decimal_part = (abs(number) - int_part)*100;   //[0-99]

    uint8_t NH = (int_part)>>8;                //Number High Byte
    uint8_t NL = (int_part) & 0x00FF;          //Number Low Byte
    uint8_t D = (int)decimal_part;             //Decimal part (7 bits)
    uint8_t SD = D;                                 //Sign and Decimal Byte
    if (number<=0) SD = D|0b10000000;               //Sign bit

    encode_bytes[0] = NH;
    encode_bytes[1] = NL;
    encode_bytes[2] = SD;
}

void encode(uint8_t id, float data[], uint8_t packet[])
{
    uint8_t num1_bytes[3];
    uint8_t num2_bytes[3];
    uint8_t num3_bytes[3];
    uint8_t num4_bytes[3];
    bytesEncode(data[0], num1_bytes);
    bytesEncode(data[1], num2_bytes);
    bytesEncode(data[2], num3_bytes);
    bytesEncode(data[3], num4_bytes);

    packet[0] = id;

    packet[1] = num1_bytes[0];
    packet[2] = num1_bytes[1];
    packet[3] = num1_bytes[2];

    packet[4] = num2_bytes[0];
    packet[5] = num2_bytes[1];
    packet[6] = num2_bytes[2];

    packet[7] = num3_bytes[0];
    packet[8] = num3_bytes[1];
    packet[9] = num3_bytes[2];

    packet[10] = num4_bytes[0];
    packet[11] = num4_bytes[1];
    packet[12] = num4_bytes[2];

    packet[13] = checksum(packet, 14);
}

void send_data(uint8_t id, float D1, float D2, float D3, float D4)
{
    float data[4] = {D1, D2, D3, D4};
    uint8_t frame_test[14];
    encode(id, data, frame_test);
    sendFrame(frame_test, 14);
}