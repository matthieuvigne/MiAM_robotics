#ifndef VISION_SHM_H
#define VISION_SHM_H

/// Define a shared memory between vision process and main robot process
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cstring>

#include <iostream>

#define MAX_MARKERS 10

struct Marker
{
  double radius;
  double theta_rad;
  double height;
}  __attribute__((__packed__));

struct VisionBuffer
{
    int nMarkers;
    Marker markers[MAX_MARKERS];
}  __attribute__((__packed__));


#define STORAGE_ID "/shm_vision"


class SHMWriter
{
    public:
        bool init()
        {
			addr_ = MAP_FAILED;
			shm_unlink(STORAGE_ID);
            int fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            if (fd == -1)
				return false;

            // extend shared memory object as by default it's initialized with size 0
            int res = ftruncate(fd, sizeof(VisionBuffer));
            if (res == -1)
				return false;

            // map shared memory to process address space
            addr_ = mmap(NULL, sizeof(VisionBuffer), PROT_WRITE, MAP_SHARED, fd, 0);

            if (addr_ == MAP_FAILED)
                return false;
            return true;
        }

        void update(VisionBuffer const& buffer)
        {
            if (addr_ != MAP_FAILED)
                memcpy(addr_, (void*)&buffer, sizeof(VisionBuffer));
        }

        ~SHMWriter()
        {
            if (addr_ != MAP_FAILED)
			{
                munmap(addr_, sizeof(VisionBuffer));
				shm_unlink(STORAGE_ID);
			}
        }

    private:
        void *addr_; ///< SHM address
};


class SHMReader
{
    public:
        bool init()
        {
			addr_ = MAP_FAILED;
            int fd = shm_open(STORAGE_ID, O_RDONLY, S_IRUSR | S_IWUSR);
            if (fd == -1)
				return false;

            // map shared memory to process address space
            addr_ = mmap(NULL, sizeof(VisionBuffer), PROT_READ, MAP_SHARED, fd, 0);

            if (addr_ == MAP_FAILED)
                return false;
            return true;
        }

        void update(VisionBuffer &buffer)
        {
            if (addr_ != MAP_FAILED)
                memcpy((void*)&buffer, addr_, sizeof(VisionBuffer));
        }
    private:
        void *addr_; ///< SHM address
};


#endif