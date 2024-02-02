#ifndef __POSIX_SOCKET_TEMPLATE_H__
#define __POSIX_SOCKET_TEMPLATE_H__


/*
    A template for opening a non-blocking POSIX socket.
*/
int open_nb_socket(const char* addr, const char* port);

#endif
