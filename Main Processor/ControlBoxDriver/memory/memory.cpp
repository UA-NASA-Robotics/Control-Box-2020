#include "memory.hpp"
#include "memory_variables.hpp"

#ifndef nullptr
#define nullptr 0
#endif

static Variable variables [Memory::SIZE];

/*------------------------- Variable Implementation --------------------------*/

void Variable::initialize ()
{
  value = 0;
  next = nullptr;
  prev = nullptr;
}

/*-------------------------- Memory Implementation ---------------------------*/

void Memory::initialize ()
{
  for (uint16_t i = 0; i < Memory::SIZE; ++i)
    variables[i].initialize();
}

void Memory::write (uint8_t i, uint16_t value)
{
  Variable * ptr = &variables[i];
  while (ptr->prev)
    ptr = ptr->prev;
  while (ptr)
  {
    ptr->value = value;
    ptr = ptr->next;
  }
}

uint16_t Memory::read (uint8_t i)
{
  return variables[i].value;
}

void Memory::link (uint8_t i, uint8_t j)
{
  Variable * ptr = &variables[i];
  while (ptr->next)
    ptr = ptr->next;
  ptr->next = &variables[j];
}

bool Memory::valid_address (uint8_t address)
{
  return address < Memory::SIZE;
}
