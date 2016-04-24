#define __ASSERT_USE_STDERR

#include <assert.h>
#include <SerialDXL.h>

MMap::UInt8NV foo_nv(0, 0, MMap::Access::RW, 10, 60, 13);

MMap::UInt8 foo(1, MMap::Access::RW, 10, 200, 25);
MMap::UInt8 bar(2, MMap::Access::RW, 10, 200, 50);

#define N 3
uint8_t mmap_buf[] = {0x00, 0x00, 0x00, 0x00};
MMap::VariablePtr var_list[3];

MMap::MMap mem_map(3);


void setup() {
  Serial.begin(9600);
  pinMode(7, INPUT);

  mem_map.init(mmap_buf, var_list);

  mem_map.registerVariable(&foo_nv);
  mem_map.registerVariable(&foo);
  mem_map.registerVariable(&bar);

  mem_map.load(); // Load EEPROM Vars
  mem_map.serialize();
  Serial.println("INIT VALUES");
  Serial.println(foo_nv.data);
  Serial.println(foo.data);
  Serial.println(bar.data);
  printBuf();
  /*
   * TEST RESET
   */
  delay(50);
  if (digitalRead(7) == HIGH)
  {
    Serial.println("TEST RESET");
    mem_map.reset();
    mem_map.serialize();
    printBuf();
    assert(mmap_buf[0] == 13);
    assert(mmap_buf[1] == 25);
    assert(mmap_buf[2] == 50);
    Serial.println("TEST RESET [OK]");
  }

  /*
   * TEST DATA MOD
   */
  Serial.println("TEST DATA MOD");
  foo_nv.data = 50;
  foo.data = 15;
  bar.data = 180;
  mem_map.serialize();
  printBuf();
  assert(mmap_buf[0] == 50);
  assert(mmap_buf[1] == 15);
  assert(mmap_buf[2] == 180);
  Serial.println("TEST DATA MOD [OK]");

  /*
   * TEST DESERIALIZE
   */
  Serial.println("TEST DESERIALIZE");
  mmap_buf[0] = 48;
  mmap_buf[1] = 19;
  mmap_buf[2] = 192;
  mem_map.deserialize();
  printBuf();
  assert(foo_nv.data == 48);
  assert(foo.data == 19);
  assert(bar.data == 192);
  Serial.println("TEST DESERIALIZE [OK]");

}

void printBuf()
{
  Serial.println("Buffer: ");
  for (uint8_t i = 0; i < N; ++i)
  {
    Serial.print(mmap_buf[i], DEC); Serial.print('|');
  }
  Serial.print('\n');
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
