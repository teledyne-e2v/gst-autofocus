#pragma once

#include <stdbool.h>
#include <stdlib.h>

typedef struct log
{
    char *message;
    bool isValid;
    struct log *next;
} Log;

typedef struct list
{
    Log *head;
    size_t len;
} List;

Log *createNewEntry(char *message, size_t msgLen);

/**
 * @brief Insert a new message at the end of the list
 * 
 * @param list      The list
 * @param message   The message to append
 */
void insert(List *list, char *message);

/**
 * @brief Free the entire list
 * 
 * @param list  The list
 */
void freeList(List *list);

/**
 * @brief Mark each message as invalid to reuse the already allocated memory
 * 
 * @param list The list
 */
void invalidList(List *list);

/**
 * @brief Get the list as a string
 * 
 * @param list      The list
 * @return char*    The string containing all log information
 */
char *getListStr(List *list);
