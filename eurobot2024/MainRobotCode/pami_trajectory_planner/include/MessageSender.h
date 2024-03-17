#ifndef MESSAGE_SENDER
#define MESSAGE_SENDER

namespace message_sender
{
    void send_message(float* message, int message_size_in_float_number, const char* str_ip_addr);
    void send_message_udp(float* message, int message_size_in_float_number, const char* str_ip_addr);
}

#endif