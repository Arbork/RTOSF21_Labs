/*
 * queue
 *
 *  Created on: Oct 27, 2021
 *      Author: Alexander Bork
 */

#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

struct queue{
    struct node_t * head;
    struct node_t * last;
};

struct queue testQueue;

struct node_t* create_queue(void) {
    struct node_t * head = create_new_node(0);
    testQueue.head = head;
    return testQueue.head;
}

struct node_t* create_new_node(uint8_t message) {
    struct node_t * newNode = malloc(sizeof(struct node_t));
    newNode->msg = message;
    newNode->next = NULL;
    return newNode;
}

uint8_t peek(struct node_t** head) {
    return (*head)->next->msg;
}

void pop(struct node_t** head) {
    struct node_t * temp;   // Holds the current element at the top of the queue
    struct node_t * next;   // Holds the element 2nd in the queue
    temp = (*head)->next;   // Temp is the first ele of the queue
    next = temp->next;      // Next is the 2nd ele of the queue
    (*head)->next = next;   // Set the new top of the queue to be the previous 2nd
    free(temp);             // Free up the top of the queue
}

void push(struct node_t** head, uint8_t message){

    struct node_t * newNode = create_new_node(message);
    struct node_t * temp1 = (*head);        // Point to the head
    struct node_t * temp2 = temp1->next;    // Point to the 1st node in the queue
    while(temp2 != NULL){                   // While the next node is not null, parse through the queue
        temp1 = temp2;
        temp2 = temp2->next;
    }
    temp2 = newNode;
    temp1->next = temp2;
    testQueue.last = temp2;
}

uint8_t is_empty(struct node_t** head) {
    if((*head)->next == NULL){
        return 1;
    }
    return 0;
}

void empty_queue(struct node_t** head) {
    while(is_empty(head) != 1){
        pop(head);
    }
}
