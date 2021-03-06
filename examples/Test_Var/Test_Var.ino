#define __ASSERT_USE_STDERR

#include <assert.h>
#include "SerialDXL.h"
#include "types/UInt8.h"
#include "types/Int32.h"

MMap::UInt8 foo(MMap::Access::RW, MMap::Storage::RAM, 10, 200, 25);
MMap::UInt8 bar(MMap::Access::R, MMap::Storage::RAM, 10, 200, 50);

MMap::UInt8 ep(MMap::Access::RW, MMap::Storage::EEPROM, 10, 190, 30);

MMap::MMap mem_map(3);

void setup() {
  Serial.begin(9600);
  pinMode(7, INPUT);

  foo.data = 38;
  mem_map.registerVariable(&foo);
  mem_map.registerVariable(&bar);
  mem_map.registerVariable(&ep);
  
  Serial.print("\teepromOffset_ "); Serial.println(mem_map.eepromOffset_);
  Serial.print("\tramOffset_ "); Serial.println(mem_map.ramOffset_);

  mem_map.init();

  if (digitalRead(7))
  {
    Serial.println("Saving data into EEPROM");
    mem_map.save();
  }
  
  
  mem_map.load();

  Serial.println("Init");
  Serial.println(foo.data);
  Serial.println(bar.data);
  Serial.println(ep.data);
  
  Serial.println("MMap buffer");
  Serial.print("\tbufN_ "); Serial.println(mem_map.bufN_);
  Serial.print("\tvarN_ "); Serial.println(mem_map.varN_);
  Serial.print("\tramOffset_ "); Serial.println(mem_map.ramOffset_);
  Serial.print("\tBuffer ");
    Serial.print(mem_map.msgBuffer_[0]); Serial.print("|");
    Serial.print(mem_map.msgBuffer_[1]); Serial.print("|");
  
  mem_map.set(0, 5);
  mem_map.set(1, 210);
  mem_map.set(2, 195);
  Serial.print("\tBuffer ");
    Serial.print(mem_map.msgBuffer_[0]); Serial.print("|");
    Serial.print(mem_map.msgBuffer_[1]); Serial.print("|");
  
  mem_map.deserialize();
  Serial.println("\nRecover values");
  Serial.println(foo.data);
  Serial.println(bar.data);
  Serial.println(ep.data);
}

void loop() {
  delay(1000);
}

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  // transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.
  abort();
}
