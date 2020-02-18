#include "message_queue.hpp"

MessageQueue::MessageQueue ()
{
  head = 0;
  tail = 0;
  message_count = 0;
}

void MessageQueue::push (const Message & msg)
{
  queue[tail++] = msg;
  if (tail >= SIZE)
    tail = 0;
  ++message_count;
}

Message MessageQueue::pop ()
{
  Message & msg = queue[head++];
  if (head >= SIZE)
    head = 0;
  --message_count;
  return msg;
}

bool MessageQueue::empty () const
{
  return message_count == 0;
}

bool MessageQueue::full () const
{
  return message_count >= SIZE;
}

uint8_t MessageQueue::count () const
{
  return message_count;
}