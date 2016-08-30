#include "SerialDXL.h"

MMap::Variable<UInt32, UInt32::type, 0, 200, 10> a(MMap::Access::RW, MMap::Storage::RAM);
MMap::Variable<UInt8, UInt8::type, 0, 200, 10> b(MMap::Access::RW, MMap::Storage::RAM);
MMap::Variable<UInt8, UInt8::type, 0, 200, 10> c(MMap::Access::RW, MMap::Storage::EEPROM);

MMap::MMap mmap(3);

void setup() {
  Serial.begin(115200);
  pinMode(7, INPUT);

  mmap.registerVariable(&a);
  mmap.registerVariable(&b);
  mmap.registerVariable(&c);
  mmap.init();

  if(digitalRead(7)==HIGH)
  {
    Serial.println("MMap reset");
    mmap.reset();
  }
  
  mmap.load();
  
  // Info
  Serial.print("MMap var size: "); Serial.println(mmap.varCount_);
  Serial.print("MMap size: "); Serial.println(mmap.bufN_);


}

void loop() {
  // Test
  a.data = 5U;
  b.data = 28U;
  uint8_t ser = mmap.serialize();
  Serial.print("MMap serialize: "); Serial.println(ser);
  Serial.print("MMap buffer: "); mmap.printBuffer();

  delay(1000);
  
}

