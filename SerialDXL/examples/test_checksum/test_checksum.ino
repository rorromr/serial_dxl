// Mesage reception state
uint8_t msgState_=0;
uint8_t msgParamIdx_=2;
uint8_t msgLen_=0;
uint8_t msgFinish_=0;
uint8_t msgChecksum_=0;
// Buffer
#define BUF_SIZE 10
uint8_t rxMsgBuf_[BUF_SIZE];
// ID
uint8_t ID = 1;

void print_buffer(int n)
{
  int i;
  Serial.print("[");
  for (i = 0; i < n-4; ++i)
  {
    Serial.print(rxMsgBuf_[i], HEX);
    Serial.print(",");
  }
  Serial.print(rxMsgBuf_[n-4], HEX);
  Serial.print("] Finish: ");
  Serial.println(msgFinish_);
}

void process(uint8_t data)
{
  switch(msgState_)
  {
    case 0: // 0xFF
      msgState_ = data == 0xFF ? 1 : 0;
      break;
      
    case 1: // 0XFF
      msgState_ = data == 0xFF ? 2 : 1;
      break;
      
    case 2: // ID
      // Check error
      msgState_ = data == ID ? 3 : 0;
      break;
      
    case 3: // Length
      msgLen_ = data;
      // Checksum
      msgChecksum_ += ID + data;
      // Save length in the RX message buffer
      rxMsgBuf_[0] = data;
      msgState_ = 4;
      break;

    case 4: // Instruction
      // Save instruction in the RX message buffer
      rxMsgBuf_[1] = data;
      // Checksum
      msgChecksum_ += data;
      //printf("%i\n", msgChecksum_);
      // Check for short message
      msgState_ = msgLen_ <= 2 ? 6 : 5;
      break;
    
    case 5: // Parameters
      rxMsgBuf_[msgParamIdx_++] = data;
      // Checksum
      msgChecksum_ += data;
      //printf("0x%.8X\n", msgChecksum_);
      // Check message length
      msgState_ = msgParamIdx_ >= msgLen_ ? 6 : msgState_;
      break;

    case 6: // Checksum
      Serial.print(data, HEX);
      Serial.print(" ");
      Serial.print((uint8_t)(~((uint8_t)((msgChecksum_)&0xff))), HEX);
      Serial.print(" ");
      Serial.print(~(msgChecksum_&0xff), HEX);
      Serial.print(" ");
      Serial.print(~(msgChecksum_), HEX);
      Serial.print(" ");
      Serial.print((uint8_t)~(msgChecksum_), HEX);
      Serial.print(" ");
      Serial.print(~(msgChecksum_)==data, HEX);
      Serial.print(" ");
      Serial.print((uint8_t)(~msgChecksum_)==data, HEX);
      rxMsgBuf_[msgParamIdx_] = data;
      // Check checksum
      msgFinish_ = ((uint8_t)(~msgChecksum_))==data;
      // Reset states
      msgState_ = 0;
      msgParamIdx_ = 2;
      msgLen_ = 0;
      msgChecksum_ = 0;
      break;
  }
}

void reset()
{
  msgState_=0;
  msgParamIdx_=2;
  msgLen_=0;
  msgFinish_=0;
  msgChecksum_=0;
}

void init_buffer()
{
  int i;
  for (i = 0; i < BUF_SIZE; ++i)
  {
    rxMsgBuf_[i] = 0;
  }
}

void exec_test(uint8_t *test, uint8_t length)
{
  int i;
  for (i = 0; i < length; ++i)
  {
    process(test[i]);
  }
}

int run() {
  // Init buffer
  uint8_t test1[] = {0xff, 0xff, 0x01, 0x03, 0x01, 0x01, 0xf9};
  uint8_t test2[] = {0xff, 0xff, 0x01, 0x04, 0x02, 0x2b, 0x01, 0xcc};
  uint8_t test3[] = {0xff, 0xff, 0x01, 0x04, 0x02, 0x2b, 0x01, 0xcc};
  uint8_t test4[] = {0xff, 0xff, 0x01, 0x03, 0x00, 0x20, 0xdb};
  uint8_t test5[] = {0xff, 0xff, 0x01, 0x04, 0x03, 0x03, 0x01, 0xf3};// Fallo ID
  uint8_t test6[] = {0xff, 0xff, 0x01, 0x04, 0x03, 0x03, 0x01, 0xf3};

  uint8_t* tests[] = {test1, test2, test3, test4, test5, test6};
  uint8_t lengths[] = {7,8,8,7,8,8};
  uint8_t results[] = {1,1,1,1,1,1};

  init_buffer();
  int i;
  for(i=0; i<6; ++i)
  {
    Serial.print("TEST ");
    Serial.print(i);
    Serial.print(": ");
    exec_test(tests[i], lengths[i]);
    if (msgFinish_ == results[i])
    {
      Serial.println("[OK]");
    }
    else
    {
      Serial.println("[FAILED]");
      print_buffer(lengths[i]);
    }
    reset();
    init_buffer();
  }
  return(0);
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("[START]");
  run();
}

void loop() {
  // put your main code here, to run repeatedly:
  //run();
}
