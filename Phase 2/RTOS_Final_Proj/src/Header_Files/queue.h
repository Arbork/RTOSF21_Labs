/*
 * queue
 *
 *  Created on: Oct 27, 2021
 *      Author: Alexander Bork
 */

#include <stdio.h>
#include <stdlib.h>

#ifndef __QUEUE__
#define __QUEUE__

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the Queue's Node information
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t {
    // Task information
    uint8_t msg;

    // Pointer to the next node in the queue
    struct node_t* next;
};


//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Creates a queue.
///
/// @param[in] task The task information
/// @param[in] size The size of the task array
///
/// @return the head of the new queue
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_queue(void);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Create a new node for the queue
///
/// @param task The task information
///
/// @return a newly allocated task
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_new_node(uint8_t message);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Returns the top node in the queue
///
/// @param head The head of the queue
///
/// @return the task at the top of the queue
//----------------------------------------------------------------------------------------------------------------------------------
uint8_t peek(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Removes the element at the top of the queue.
///
/// @param head The head of the queue.
//----------------------------------------------------------------------------------------------------------------------------------
void pop(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Push a new task into the queue
///
/// @param head The head of the queue
/// @param task The task to be put into the queue
//----------------------------------------------------------------------------------------------------------------------------------
void push(struct node_t** head, uint8_t message);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Determines whether the specified head is empty.
///
/// @param head The head of the Queue
///
/// @return True if the specified head is empty, False otherwise.
//----------------------------------------------------------------------------------------------------------------------------------
uint8_t is_empty(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Remove all items from the queue
///
/// @param head The head of the queue
//----------------------------------------------------------------------------------------------------------------------------------
void empty_queue(struct node_t** head);

#endif // __QUEUE__
