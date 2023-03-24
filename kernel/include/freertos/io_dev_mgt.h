/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-29     zhengqian    the first version
 */

#ifndef __JEE_DevMgt_H__
#define __JEE_DevMgt_H__

#ifdef __cplusplus
extern "C" {
#endif


/**
 * jee_container_of - return the start address of struct type, while ptr is the
 * member of struct type.
 */
#define jee_container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))


/**
 * @brief initialize a list device
 */
#define JEE_LIST_DEVICE_INIT(device) { &(device), &(device) }

/**
 * @brief initialize a list
 *
 * @param l list to be initialized
 */
jee_inline void jee_list_init(jee_list_t *l)
{
    l->next = l->prev = l;
}

/**
 * @brief insert a node after a list
 *
 * @param l list to insert it
 * @param n new node to be inserted
 */
jee_inline void jee_list_insert_after(jee_list_t *l, jee_list_t *n)
{
    l->next->prev = n;
    n->next = l->next;

    l->next = n;
    n->prev = l;
}

/**
 * @brief insert a node before a list
 *
 * @param n new node to be inserted
 * @param l list to insert it
 */
jee_inline void jee_list_insert_before(jee_list_t *l, jee_list_t *n)
{
    l->prev->next = n;
    n->prev = l->prev;

    l->prev = n;
    n->next = l;
}

/**
 * @brief remove node from list.
 * @param n the node to remove from the list.
 */
jee_inline void jee_list_remove(jee_list_t *n)
{
    n->next->prev = n->prev;
    n->prev->next = n->next;

    n->next = n->prev = n;
}

/**
 * @brief tests whether a list is empty
 * @param l the list to test.
 */
jee_inline int jee_list_isempty(const jee_list_t *l)
{
    return l->next == l;
}

/**
 * @brief get the list length
 * @param l the list to get.
 */
jee_inline unsigned int jee_list_len(const jee_list_t *l)
{
    unsigned int len = 0;
    const jee_list_t *p = l;
    while (p->next != l)
    {
        p = p->next;
        len ++;
    }

    return len;
}

/**
 * @brief get the struct for this entry
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define jee_list_entry(node, type, member) \
    jee_container_of(node, type, member)

/**
 * jee_list_for_each - iterate over a list
 * @pos:    the jee_list_t * to use as a loop cursor.
 * @head:   the head for your list.
 */
#define jee_list_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * jee_list_for_each_safe - iterate over a list safe against removal of list entry
 * @pos:    the jee_list_t * to use as a loop cursor.
 * @n:      another jee_list_t * to use as temporary storage
 * @head:   the head for your list.
 */
#define jee_list_for_each_safe(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

/**
 * jee_list_for_each_entry  -   iterate over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:   the head for your list.
 * @member: the name of the list_struct within the struct.
 */
#define jee_list_for_each_entry(pos, head, member) \
    for (pos = jee_list_entry((head)->next, typeof(*pos), member); \
         &pos->member != (head); \
         pos = jee_list_entry(pos->member.next, typeof(*pos), member))

/**
 * jee_list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:    the type * to use as a loop cursor.
 * @n:      another type * to use as temporary storage
 * @head:   the head for your list.
 * @member: the name of the list_struct within the struct.
 */
#define jee_list_for_each_entry_safe(pos, n, head, member) \
    for (pos = jee_list_entry((head)->next, typeof(*pos), member), \
         n = jee_list_entry(pos->member.next, typeof(*pos), member); \
         &pos->member != (head); \
         pos = n, n = jee_list_entry(n->member.next, typeof(*n), member))

/**
 * jee_list_first_entry - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:   the type of the struct this is embedded in.
 * @member: the name of the list_struct within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define jee_list_first_entry(ptr, type, member) \
    jee_list_entry((ptr)->next, type, member)

#define JEE_SLIST_DEVICE_INIT(device) { JEE_NULL }

/**
 * @brief initialize a single list
 *
 * @param l the single list to be initialized
 */
jee_inline void jee_slist_init(jee_slist_t *l)
{
    l->next = JEE_NULL;
}

jee_inline void jee_slist_append(jee_slist_t *l, jee_slist_t *n)
{
    struct jee_slist_node *node;

    node = l;
    while (node->next) node = node->next;

    /* append the node to the tail */
    node->next = n;
    n->next = JEE_NULL;
}

jee_inline void jee_slist_insert(jee_slist_t *l, jee_slist_t *n)
{
    n->next = l->next;
    l->next = n;
}

jee_inline unsigned int jee_slist_len(const jee_slist_t *l)
{
    unsigned int len = 0;
    const jee_slist_t *list = l->next;
    while (list != JEE_NULL)
    {
        list = list->next;
        len ++;
    }

    return len;
}

jee_inline jee_slist_t *jee_slist_remove(jee_slist_t *l, jee_slist_t *n)
{
    /* remove slist head */
    struct jee_slist_node *node = l;
    while (node->next && node->next != n) node = node->next;

    /* remove node */
    if (node->next != (jee_slist_t *)0) node->next = node->next->next;

    return l;
}

jee_inline jee_slist_t *jee_slist_first(jee_slist_t *l)
{
    return l->next;
}

jee_inline jee_slist_t *jee_slist_tail(jee_slist_t *l)
{
    while (l->next) l = l->next;

    return l;
}

jee_inline jee_slist_t *jee_slist_next(jee_slist_t *n)
{
    return n->next;
}

jee_inline int jee_slist_isempty(jee_slist_t *l)
{
    return l->next == JEE_NULL;
}

/**
 * @brief get the struct for this single list node
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define jee_slist_entry(node, type, member) \
    jee_container_of(node, type, member)

/**
 * jee_slist_for_each - iterate over a single list
 * @pos:    the jee_slist_t * to use as a loop cursor.
 * @head:   the head for your single list.
 */
#define jee_slist_for_each(pos, head) \
    for (pos = (head)->next; pos != JEE_NULL; pos = pos->next)

/**
 * jee_slist_for_each_entry  -   iterate over single list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:   the head for your single list.
 * @member: the name of the list_struct within the struct.
 */
#define jee_slist_for_each_entry(pos, head, member) \
    for (pos = jee_slist_entry((head)->next, typeof(*pos), member); \
         &pos->member != (JEE_NULL); \
         pos = jee_slist_entry(pos->member.next, typeof(*pos), member))

/**
 * jee_slist_first_entry - get the first element from a slist
 * @ptr:    the slist head to take the element from.
 * @type:   the type of the struct this is embedded in.
 * @member: the name of the slist_struct within the struct.
 *
 * Note, that slist is expected to be not empty.
 */
#define jee_slist_first_entry(ptr, type, member) \
    jee_slist_entry((ptr)->next, type, member)

/**
 * jee_slist_tail_entry - get the tail element from a slist
 * @ptr:    the slist head to take the element from.
 * @type:   the type of the struct this is embedded in.
 * @member: the name of the slist_struct within the struct.
 *
 * Note, that slist is expected to be not empty.
 */
#define jee_slist_tail_entry(ptr, type, member) \
    jee_slist_entry(jee_slist_tail(ptr), type, member)

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
