#include<iostream>
#include<string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
using raw_type = void;

#define PORT 12000
#define IP "127.0.0.1"

struct __attribute__ ((packed)) data{
    int id;
    int8_t age;
    // const char *name;
};

int main()
{
    int sockfd, n;
    struct sockaddr_in serv_addr;
    char buffer[256];

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0){
        perror("Error open socket");
        exit(1);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    
    data my_data;
    my_data.id = 1000;
    my_data.age = 10;
    // std::string name = "test";
    // my_data.name = name.c_str();

    sendto(sockfd, 
            reinterpret_cast<raw_type *>(&my_data),
            sizeof(my_data), 0,
            (struct sockaddr *)&serv_addr, sizeof(sockaddr));
    std::cout << "send one message with: " << sizeof(my_data) << std::endl;
    return 0;
}