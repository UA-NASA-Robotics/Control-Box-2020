#ifndef MESSAGE_QUEUE_HPP
#define MESSAGE_QUEUE_HPP
#include "message.hpp"

class MessageQueue
{
private:
  static const uint8_t SIZE = 50;
public:
  MessageQueue ();
  void push (const Message &);
  Message pop ();
  bool empty () const;
  bool full () const;
  uint8_t count () const;
private:
  uint8_t head;
  uint8_t tail;
  uint8_t message_count;
  Message queue [SIZE];
};

#endif // MESSAGE_QUEUE_HPP