/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__FSL_OS_ABSTRACTION_UCOSIII_H__)
#define __FSL_OS_ABSTRACTION_UCOSIII_H__

#include "os.h"

/*!
* @addtogroup os_abstraction_ucosiii
* @{
*/

/*******************************************************************************
 * Declarations
 ******************************************************************************/
/*! @brief Constant to pass as timeout value in order to wait indefinitely. */
#define OSA_WAIT_FOREVER 0xFFFFFFFFU

/*! @brief OSA's time range in millisecond, OSA time wraps if exceeds this value. */
#define FSL_OSA_TIME_RANGE 0xFFFFFFFFU

/*! @brief The default interrupt handler installed in vector table. */
#define OSA_DEFAULT_INT_HANDLER  ((osa_int_handler_t)(&DefaultISR))

extern void DefaultISR(void);

/*! @brief Type for an event group object in µCOS-III */
typedef struct EventUCOSIII{
    OS_FLAG_GRP            group;      /*!< µCOS-III's event entity    */
    osa_event_clear_mode_t clearMode;  /*!< Auto clear or manual clear */
} event_ucosiii;

/*! @brief Type for message queue in µCOS-III */
typedef struct MsgqUCOSIII{
    OS_Q      queue; /*!< the message queue's control block                       */
    OS_MEM    mem;   /*!< control block for the memory where save the messages    */
    void     *msgs;  /*!< pointer to the memory where save the messages           */
    uint16_t  size;  /*!< size of the message in words                            */
} msgq_struct_ucosiii;

/*! @brief Type for a task handler, returned by the OSA_TaskCreate function. */
typedef OS_TCB* task_handler_t;

/*! @brief Type for a task stack. */
typedef CPU_STK task_stack_t;

/*! @brief Type for task parameter */
typedef void* task_param_t;

/*! @brief Type for a task pointer */
typedef void (* task_t)(task_param_t param);

/*! @brief Type for a semaphore. */
typedef OS_SEM   semaphore_t;

/*! @brief Type for a mutex. */
typedef OS_MUTEX mutex_t;

/*! @brief Type for an event flags group. */
typedef OS_FLAGS event_flags_t;

/*! @brief Type for an event object. */
typedef event_ucosiii  event_t;

/*! @brief Type for a message queue. */
typedef msgq_struct_ucosiii msg_queue_t;

/*! @brief Type for a message queue handler. */
typedef msg_queue_t* msg_queue_handler_t;


/*!
* @name Thread management
* @{
*/

/*!
 * @brief Defines a task.
 *
 * This macro defines resources for a task statically. Then, the OSA_TaskCreate 
 * creates the task based on these resources.
 *
 * @param task The task function.
 * @param stackSize The stack size this task needs in bytes.
 */
#define OSA_TASK_DEFINE(task, stackSize)                          \
    OS_TCB TCB_##task;                                            \
    task_stack_t task##_stack[(stackSize)/sizeof(task_stack_t)];  \
    task_handler_t task##_task_handler = &(TCB_##task)
    
/*!
 * @brief To provide unified task piority for upper layer, OSA layer makes conversion.
 * uC/OS-III's tick task should have a high piority, so we set tick task to
 * piority 0, applications use other priorities.
 */
#define PRIORITY_OSA_TO_RTOS(osa_prio)   ((osa_prio)+1U)
#define PRIORITY_RTOS_TO_OSA(rtos_prio)  ((rtos_prio)-1U)    

/* @} */

/*!
* @name Message queues
* @{
*/

/*!
 * @brief This macro statically reserves the memory required for the queue.
 *
 * @param name Identifier for the memory region.
 * @param number Number of elements in the queue.
 * @param size Size of every elements in words.
 */
#define MSG_QUEUE_DECLARE(name, number, size)            \
    uint32_t msgs_##name[number*size];                   \
    msg_queue_t memory_##name = {                        \
        .msgs = msgs_##name                              \
    };                                                   \
    msg_queue_t *name = &(memory_##name)

/* @} */

/*! @}*/

#endif /* __FSL_OS_ABSTRACTION_UCOSIII_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

