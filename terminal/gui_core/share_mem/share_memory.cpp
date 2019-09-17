#include "share_memory.h"
 
CShareMemory::CShareMemory(const char *name, u32 size):m_memSize(size)
{	
	m_sem = sem_open(name, O_CREAT, 0X777, 1);
	if(SEM_FAILED == m_sem)
	{
		printf("sem_open error");
	}
	sem_unlink(name);
 
	SemLock();
	m_shareMemory = (u8 *)CreateShareMemory(name, m_memSize);
	SemUnLock();
}
 
/*
 * brief:  写入数据到共享内存
 * param1: buffer 写入数据buf
 * param2: length 写入数据长度
 * return: 返回写入数据的长度
**/
int CShareMemory::PutToShareMem(const u8 *buffer, u32 length)
{
	if(m_memSize < length)
	{
		printf("Input length[%u] exceeds memory length[%u] error", length, m_memSize);
		return -1;
	}
 
	SemLock();	
	memcpy(m_shareMemory, buffer, length);	
	SemUnLock();	
	return length;
}
 
/*
 * brief:  从共享内存读取数据
 * param1: buffer 读取数据buf
 * param2: length 读取数据长度
 * return: 返回读取数据的长度
**/
int CShareMemory::GetFromShareMem(u8 *buffer, u32 length)
{	
	if(m_memSize < length) // 获取数据越界
	{
		length = m_memSize;  
	}
	
	SemLock();
	memcpy(buffer, m_shareMemory, length);
	SemUnLock();
	return length;		
}
 
/*
 * brief:  创建共享内存
 * param1: name 
 * size:   创建内存大小
**/
void * CShareMemory::CreateShareMemory(const char *name, u32 size)
{
	int fd, shmid;
	void *memory;
	struct shmid_ds buf;
	char filename[32];
 
	snprintf(filename, 32, "/tmp/.%s", name);
	if((fd = open(filename, O_RDWR|O_CREAT|O_EXCL)) > 0)
	{
		close(fd);
	}
	
	shmid = shmget(ftok(filename, 'g'), size, IPC_CREAT | 0777);
	if (-1 == shmid)
    {
        perror("shmget err");
        return NULL;
    }
    //printf("shmid:%d \n", shmid);
	
	memory = shmat(shmid, NULL, 0);
	if ((void *)-1 == memory)
    {
        perror("shmget err");
        return NULL;
    }
	
	shmctl(shmid, IPC_STAT, &buf);
	if (buf.shm_nattch == 1)
	{
		memset(memory, 0, size);
	}	
	
	return memory;
}
 
CShareMemory::~CShareMemory()
{
	shmdt(m_shareMemory);
	sem_close(m_sem);
}

