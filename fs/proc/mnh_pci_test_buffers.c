/*
 *
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program;
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Larkin Kinsella <larkin.kinsella@intel.com>
 */

#include <linux/module.h>    // included for all kernel modules
#include <linux/kernel.h>    // included for KERN_INFO
#include <linux/init.h>      // included for __init and __exit macros
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>

#include <linux/io.h>


#define MAX_LEN 4096
#define MAX_ALLOCATIONS 20

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corp");
MODULE_DESCRIPTION("A procfs interface for pci buffers");

loff_t seek_buffer(struct file *filp, loff_t offset, int);

ssize_t read_buffer(struct file *filp, char *buffer, size_t length, loff_t *offset);
ssize_t write_buffer( struct file *filp, const char __user *buff,size_t len, loff_t *data);

ssize_t create_buffer( struct file *filp, const char __user *buff,size_t len, loff_t *data);

ssize_t read_id(struct file *filp, char *buffer, size_t length, loff_t *offset);
ssize_t write_id( struct file *filp, const char __user *buff,size_t len, loff_t *data);

ssize_t read_size(struct file *filp, char *buffer, size_t length, loff_t *offset);
ssize_t write_size( struct file *filp, const char __user *buff, size_t len, loff_t *data);

ssize_t write_delete( struct file *filp, const char __user *buff,size_t len, loff_t *data);
ssize_t write_delete_all( struct file *filp, const char __user *buff,size_t len, loff_t *data);
ssize_t read_last_id(struct file *filp, char *buffer, size_t length, loff_t *offset);

ssize_t read_address(struct file *filp, char *buffer, size_t length, loff_t *offset);

struct id_buffer* find_buffer(int desired_id);

static int open_all(struct inode *inode, struct file *file);
static int open_buffer(struct inode *inode, struct file *file);
static int release_all(struct inode *inode, struct file *file);

static struct proc_dir_entry *proc_entry_buffer;
static struct proc_dir_entry *proc_entry_create;
static struct proc_dir_entry *proc_entry_id;
static struct proc_dir_entry *proc_entry_delete;
static struct proc_dir_entry *proc_entry_delete_all;
static struct proc_dir_entry *proc_entry_address;
static struct proc_dir_entry *proc_entry_size;

static int buffer_offset = 0;
static int rw_size = 0;
static int buffer_id = 0;
static int next_buffer_id = 0;

struct id_buffer {
  unsigned long length;
  int id;
  void __iomem * buffer;
  unsigned long phys_addr;
  struct list_head list;
};

//struct id_buffer id_buffer_list;
LIST_HEAD(id_buffer_list);

static struct file_operations fops_buffer = {
  .read = read_buffer,
  .llseek = seek_buffer,
  .write =write_buffer,
  .open = open_buffer,
  .release = release_all

};

static struct file_operations fops_size = {
  .read = read_size,
  .write =write_size,
  .open = open_all,
  .release = release_all

};

static struct file_operations fops_id = {
  .read = read_id,
  .write =write_id,
  .open = open_all,
  .release = release_all

};
static struct file_operations fops_delete = {
  .read = (void *) 0,
  .write =write_delete,
  .open = open_all,
  .release = release_all

};

static struct file_operations fops_delete_all = {
  .read = (void *) 0,
  .write =write_delete_all,
  .open = open_all,
  .release = release_all

};

static struct file_operations fops_address = {
  .read = read_address,
  .write = (void *) 0,
  .open = open_all,
  .release = release_all

};

static struct file_operations fops_create = {
  .read = read_last_id,
  .write = create_buffer,
  .open = open_all,
  .release = release_all

};



static int __init mnh_pci_test_buffers_init(void){

  int ret = 0;

  proc_entry_buffer = proc_create("pci_test_buffer", 0666, NULL, &fops_buffer);
  if (proc_entry_buffer==NULL){
    ret = -1;
    printk(KERN_INFO "buffer file not created\n");
    goto out;
  }
  proc_entry_id = proc_create("pci_test_id", 0666, NULL, &fops_id);
  if (proc_entry_id==NULL){
    ret = -1;
    printk(KERN_INFO "id file not created\n");
    goto cleanup_1;
  }
  proc_entry_delete = proc_create("pci_test_delete", 0222, NULL, &fops_delete);
  if (proc_entry_delete==NULL){
    ret = -1;
    printk(KERN_INFO "delete  file not created\n");
    goto cleanup_2;
  }
  proc_entry_create = proc_create("pci_test_create", 0666, NULL, &fops_create);
  if (proc_entry_create==NULL){
    ret = -1;
    printk(KERN_INFO "create file not created\n");
    goto cleanup_3;
  }
  proc_entry_address = proc_create("pci_test_address", 0444, NULL, &fops_address);
  if (proc_entry_address==NULL){
    ret = -1;
    printk(KERN_INFO "address file not created\n");
    goto cleanup_4;
  }
  proc_entry_delete_all = proc_create("pci_test_delete_all", 0222, NULL, &fops_delete_all);
  if (proc_entry_delete_all==NULL){
    ret = -1;
    printk(KERN_INFO "delete all file not created\n");
    goto cleanup_5;
  }

  proc_entry_size = proc_create("pci_test_size", 0666, NULL, &fops_size);
  if (proc_entry_size ==NULL){
    ret = -1;
    printk(KERN_INFO "size file not created\n");
  }

  else{
    ret = 0;
    goto out;
  }

  remove_proc_entry("pci_test_delete_all", proc_entry_address);

 cleanup_5:
  remove_proc_entry("pci_test_address", proc_entry_address);
 cleanup_4:
  remove_proc_entry("pci_test_create", proc_entry_address);
 cleanup_3:
  remove_proc_entry("pci_test_delete", proc_entry_delete);
 cleanup_2:
  remove_proc_entry("pci_test_id", proc_entry_id);
 cleanup_1:
  remove_proc_entry("pci_test_buffer", proc_entry_buffer);
 out:
  return ret;    // Non-zero return means that the module couldn't be loaded.
}

static void __exit mnh_pci_test_buffers_cleanup(void)
{
  remove_proc_entry("pci_test_create", proc_entry_create);
  remove_proc_entry("pci_test_delete", proc_entry_delete);
  remove_proc_entry("pci_test_id", proc_entry_id);
  remove_proc_entry("pci_test_buffer", proc_entry_buffer);
  remove_proc_entry("pci_test_address", proc_entry_address);
  remove_proc_entry("pci_test_size", proc_entry_size);
  remove_proc_entry("pci_test_delete_all", proc_entry_size);



}

ssize_t create_buffer(struct file *filp, const char __user *buff,size_t len, loff_t *data)
{

  int length=0;
  unsigned long phys_addr=0;
  unsigned long memsize=0;
  char line[20];
  void *next;
  printk(KERN_INFO "write info");
  length = copy_from_user((void *)line, (void *) buff,20);
  sscanf(line, "%lx", &memsize);
  memsize = memsize < PAGE_SIZE ? PAGE_SIZE : memsize;
  if (!memsize){
    printk(KERN_WARNING "failed to get buffer size , please write size to sys file\n");
    return EINVAL;
  }
  next = kmalloc(memsize, GFP_KERNEL);

  if (!next){
    printk(KERN_WARNING "mem alloc failed\n");
    return EINVAL;
  }
  phys_addr = virt_to_phys(next);

  if (next){
    struct id_buffer *id_buffer_entry;
    id_buffer_entry = kmalloc(sizeof(struct id_buffer), GFP_KERNEL);
    if (!id_buffer_entry){
      printk(KERN_WARNING "memory allocation failed\n");
      return ENOMEM;
    }
    id_buffer_entry->phys_addr = phys_addr;
    id_buffer_entry->buffer = next;
    id_buffer_entry->id = next_buffer_id;
    id_buffer_entry->length = (int) memsize;
    INIT_LIST_HEAD(&(id_buffer_entry->list));
    buffer_id = next_buffer_id;
    printk(KERN_INFO "memory allocation size %d id %d\n", (int) memsize, next_buffer_id++);
    list_add(&(id_buffer_entry->list), &id_buffer_list);
  }
    return len;
}

ssize_t write_size(struct file *filp, const char __user *buff,size_t len, loff_t *data){
  char line[20];
  int size, length, items;
  printk(KERN_INFO "write size");
  length = copy_from_user((void *)line, (void *) buff,20);
  items = sscanf(line, "%d", &size);
  if (!items){
    printk(KERN_WARNING "failed to get size, please write integer size to sys file\n");
    return EINVAL;
  }
  else{
    printk(KERN_INFO "setting size to %d\n", size);
    rw_size = size;
  }
  return len;
}

ssize_t write_id(struct file *filp, const char __user *buff,size_t len, loff_t *data){
  char line[20];
  int id, length, items;
  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  printk(KERN_INFO "write id");
  length = copy_from_user((void *)line, (void *) buff,20);
  items = sscanf(line, "%d", &id);
  if (!items){
    printk(KERN_WARNING "failed to get chunk id, please write integer id to sys file\n");
    return EINVAL;

  }
  else if(!(buffer_entry = find_buffer(id))){
    printk(KERN_WARNING "no buffer for id %d\n", id);
    return EINVAL;
  }
  else{
    printk(KERN_INFO "setting chunk id to %d\n", id);
    buffer_id = id;
  }
  return len;
}


ssize_t write_delete(struct file *filp, const char __user *buff,size_t len, loff_t *data){
  char line[20];
  int id, length, items;
  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  printk(KERN_INFO "write delete");
  length = copy_from_user((void *)line, (void *) buff,20);
  items = sscanf(line, "%d", &id);
  if (!items){
    printk(KERN_WARNING "failed to get delete id, please write integer id to sys file\n");
    return EINVAL;
  }
  else if(!(buffer_entry = find_buffer(id))){
    printk(KERN_WARNING "no buffer for id %d\n", id);
    return EINVAL;
  }
  else{
    printk(KERN_INFO "deleting chunk id to %d\n", id);
    list_del(&(buffer_entry->list));
    kfree(buffer_entry->buffer);
    kfree(buffer_entry);
  }
  return len;
}
ssize_t write_delete_all(struct file *filp, const char __user *buff,size_t len, loff_t *data){

  struct id_buffer *temp_entry = (struct id_buffer *) 0;
  struct list_head *node;
  struct list_head *tmp;
  list_for_each_safe(node,tmp, &id_buffer_list){
    temp_entry = list_entry(node, struct id_buffer, list);
    printk(KERN_INFO "delete buffer for id %d\n", temp_entry->id);
    list_del(node);
    kfree(temp_entry->buffer);
    kfree(temp_entry);
  }

  return len;
}

struct id_buffer* find_buffer(int desired_id){
  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  struct id_buffer *temp_entry;
  struct list_head *node;
  list_for_each(node, &id_buffer_list){
      temp_entry = list_entry(node, struct id_buffer, list);
      printk(KERN_INFO "buffer for id %d\n", temp_entry->id);
      if (temp_entry->id == desired_id){
	buffer_entry = temp_entry;
	printk(KERN_INFO "retrieving buffer for id %d\n", buffer_entry->id);
	break;
      }
  }
  if (!buffer_entry){
    printk(KERN_INFO "not retrieving buffer for id\n");
  }
  return buffer_entry;
}
ssize_t write_buffer(struct file *filp, const char __user *buff, size_t len, loff_t *data){

  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  char *p;
  char *tmp_buffer;

  printk(KERN_INFO "size %d  offset %d\n",(int) len, (int) buffer_offset);
  buffer_entry = find_buffer(buffer_id);


  if (!buffer_entry){
    printk(KERN_INFO "couldn't find  buffer for id %d\n", buffer_id);
    return EINVAL;
  }
  p= (char *)buffer_entry->buffer + buffer_offset;
  if (buffer_offset + len  > buffer_entry->length){
    printk(KERN_INFO "insufficent buffer space\n");
    return EINVAL;
  }
  tmp_buffer= (void *) kmalloc(len, GFP_KERNEL);
  if (!tmp_buffer){
    printk(KERN_INFO "failed to allocate temp buffer");
    return ENOMEM;
  }


  if (copy_from_user((void *)tmp_buffer, (void *) buff, len)){
    printk(KERN_INFO "failed to copy bytes");
    return EIO;
  }
  memcpy_toio(p, tmp_buffer, len);

  kfree(tmp_buffer);
  printk(KERN_INFO "wrote %d bytes\n", (int) len);
  buffer_offset+= len;
  return len;
}





loff_t seek_buffer(struct file *filp, loff_t offset, int whence){
  return 0;
}

ssize_t read_buffer(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  int ret=0, bytes_sent=0;
  int seek = buffer_offset;
  char *test_buffer;
  void *tmp_buffer;
  long bytes_copied= 0;
  printk(KERN_INFO "length %d  offset %d\n",(int) length, seek);

  buffer_entry = find_buffer(buffer_id);

  if (buffer_entry){
    printk(KERN_INFO "buffer entry length %d ",(int) buffer_entry->length);
    printk(KERN_INFO "echoing  buffer data for id %d\n", buffer_id);
    test_buffer = (char *) buffer_entry->buffer;

    bytes_sent = length < rw_size -seek ? length : rw_size - seek;
    tmp_buffer= kmalloc(bytes_sent, GFP_KERNEL);
    if (!tmp_buffer){
      printk(KERN_INFO "failed to allocate temp buffer");
      return ENOMEM;
    }
    memcpy_fromio(tmp_buffer, (void *) test_buffer + buffer_offset,bytes_sent);
    bytes_copied=copy_to_user((void *)buffer ,tmp_buffer,bytes_sent);
    kfree(tmp_buffer);
    if (bytes_copied){
      printk(KERN_INFO "failed to copy %ld bytes", bytes_copied);
      return EINVAL;
    }


    ret = bytes_sent ;
    buffer_offset += ret ;
    printk(KERN_INFO "echoed %d bytes buffer_offset now %d\n", ret, buffer_offset);
  }
  else{
    printk(KERN_INFO "couldn't find  buffer for id %d\n", buffer_id);
    ret = EINVAL;
  }
  return ret;
}

ssize_t read_address(struct file *filp, char *buffer, size_t length, loff_t *offset){

  int val_length;
  char line [80];
  struct id_buffer *buffer_entry = (struct id_buffer *) 0;
  char *test_buffer;
  long bytes_copied= 0;
  buffer_entry = find_buffer(buffer_id);
  test_buffer = (char *) buffer_entry->buffer;
  printk(KERN_INFO "read address\n");
  sprintf(line, "virtual\t%lx\nphysical\t%lx\n", (long) test_buffer, buffer_entry->phys_addr);
  val_length = strlen(line);
  if (val_length<=(*offset)){
    return 0;
  }
  else{
    bytes_copied=copy_to_user((void *)buffer ,(void *) line, val_length);
    if (bytes_copied){
      printk(KERN_INFO "failed to copy %ld bytes", bytes_copied);
      return ERESTARTSYS;
    }

  return val_length;
  }
}

ssize_t read_size(struct file *filp, char *buffer, size_t length, loff_t *offset){

  int val_length;
  long bytes_copied;
  char line [20];
  printk(KERN_INFO "read size\n");
  sprintf(line, "%d\n", rw_size);
  val_length = strlen(line);
  if (val_length <=(*offset)){
    return 0;
  }
  else{
    bytes_copied=copy_to_user((void *)buffer ,(void *) line, val_length);
    if (bytes_copied){
      printk(KERN_INFO "failed to copy %ld bytes", bytes_copied);
      return ERESTARTSYS;
    }

  return val_length;
  }
}

ssize_t read_id(struct file *filp, char *buffer, size_t length, loff_t *offset){

  int val_length;
  long bytes_copied;
  char line [20];
  printk(KERN_INFO "read id\n");
  sprintf(line, "%d\n", buffer_id);
  val_length = strlen(line);
  if (val_length<=(*offset)){
    return 0;
  }
  else{
    bytes_copied=copy_to_user((void *)buffer ,(void *) line, val_length);
    if (bytes_copied){
      printk(KERN_INFO "failed to copy %ld bytes", bytes_copied);
      return ERESTARTSYS;
    }

  return val_length;
  }
}

ssize_t read_last_id(struct file *filp, char *buffer, size_t length, loff_t *offset){
  int val_length;
  long bytes_copied;
  //char *p;
  char line [20];
  printk(KERN_INFO "read id\n");
  sprintf(line, "%d\n", next_buffer_id - 1 );
  val_length = strlen(line);
  if (val_length<=(*offset)){
    return 0;
  }
  else{
    bytes_copied=copy_to_user((void *)buffer ,(void *) line, val_length);
    if (bytes_copied){
      printk(KERN_INFO "failed to copy %ld bytes", bytes_copied);
      return EINVAL;
    }

  return val_length;
  }

}


static int open_all(struct inode *inode, struct file *file)
{

  return 0;
}
static int open_buffer(struct inode *inode, struct file *file)
{
  buffer_offset = 0;
  return 0;
}

static int release_all(struct inode *inode, struct file *file)
{


  /* Decrement the usage count, or else once you opened the file, you'll
     never get get rid of the module. */

  return 0;
}



module_init(mnh_pci_test_buffers_init);
module_exit(mnh_pci_test_buffers_cleanup);
