#include "logger.h"

#include <string.h>
#include <stdio.h>

Log *createNewEntry(char *message, size_t msgLen)
{
    Log *tmp = (Log*)malloc(sizeof(Log));

    if (tmp == NULL)
    {
        printf("Error: unable to allocate new log entry into memory\n");
    }
    else
    {
        tmp->message = (char*)malloc(sizeof(char) * msgLen);

        if (tmp->message == NULL)
        {
            printf("Error: unable to allocate new message into memmory\n");
            free(tmp);
            tmp = NULL;
        }
        else
        {
            tmp->message = strcpy(tmp->message, message);
            tmp->isValid = true;
            tmp->next = NULL;
        }
    }

    return tmp;
}

void insert(List *list, char *message)
{
    if (list == NULL) return;

    size_t msgLen = strlen(message);

    if (list->head == NULL)
    {
        list->head = createNewEntry(message, msgLen);
        list->len += msgLen;
    }
    else
    {
        Log *curr = list->head;
        Log *prev = NULL;

        while (curr != NULL && curr->isValid)
        {
            prev = curr;
            curr = curr->next;
        }

        if (curr != NULL)
        {
            curr->message = (char*)realloc(curr->message, sizeof(char) * msgLen);

            if (curr->message == NULL)
            {
                printf("Error: Unable to reallocate memory for the new message\n");
            }
            else
            {
                curr->message = strcpy(curr->message, message);
                curr->isValid = true;
                list->len += msgLen;
            }
        }
        else
        {
            prev->next = createNewEntry(message, msgLen);
            list->len += msgLen;
        }
    }
}

void freeList(List *list)
{
    if (list == NULL) return;

    Log *curr = list->head;
    Log *tmp = NULL;

    while (curr != NULL)
    {
        tmp = curr->next;
        free(curr->message);
        free(curr);
        curr = tmp;
    }

    list->len = 0;
}

void invalidList(List *list)
{
    if (list == NULL) return;

    Log *curr = list->head;

    while (curr != NULL)
    {
        curr->isValid = false;
        curr = curr->next;
    }

    list->len = 0;
}

char *getListStr(List *list)
{
    if (list == NULL) return NULL;

    char *fullLog = (char*)malloc(sizeof(char) * (list->len + 1));
    
    if (fullLog == NULL)
    {
        printf("Error: Unable to allocate memory\n");
    }
    else
    {    
        fullLog[0] = '\0';

        Log *curr = list->head;

        while (curr != NULL && curr->isValid)
        {
            if (curr->message != NULL)
                fullLog = strcat(fullLog, curr->message);
            curr = curr->next;
        }
    }
    
    return fullLog;
}
