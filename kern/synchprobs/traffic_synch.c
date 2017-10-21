#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */

static struct lock *intersectionOrderLock;
static struct cv *cv_origins[4];

static volatile unsigned int origin_order[4];
static volatile bool origin_exists[4];
static volatile unsigned int car_counts[4];
static volatile unsigned int curr_cars = 0;
static volatile unsigned int curr = 0;
static volatile unsigned int next = 1;

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  intersectionOrderLock = lock_create("intersectionOrderLock");
  if (intersectionOrderLock == NULL) {
    panic("could not create intersection order lock");
  }

  for (int i = 0; i < 4; i++){
    cv_origins[i] = cv_create("cv");
    if (cv_origins[i] == NULL) {
      panic("could not create a condition variable");
    }

    origin_order[i] = 11;
    origin_exists[i] = false;
    car_counts[i] = 0;
  }
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersectionOrderLock != NULL);
  lock_destroy(intersectionOrderLock);
  for (int i = 0; i < 4; i++){
    KASSERT(cv_origins[i]!= NULL);
    cv_destroy(cv_origins[i]);
  }
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  (void)destination;

  lock_acquire(intersectionOrderLock);

  if(curr_cars > 0){

    if(!origin_exists[origin]){
      origin_order[next%4] = origin;
      next++;
      origin_exists[origin] = true;
    }
    car_counts[origin]++;
    cv_wait(cv_origins[origin], intersectionOrderLock);

  } else {
    curr_cars++;
  }

  lock_release(intersectionOrderLock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation witfh your own implementation */
  (void)origin;
  (void)destination;

  lock_acquire(intersectionOrderLock);

  curr_cars--;

  if(curr_cars == 0){
    /* this is the last thread from the batch; reset + broadcast */
    if(origin_order[(curr+1)%4] != 11){
      curr++;
      int curr_origin = origin_order[curr%4];
      origin_order[curr%4] = 11;
      origin_exists[curr_origin] = false;
      
      curr_cars = car_counts[curr_origin];
      car_counts[curr_origin] = 0;

      cv_broadcast(cv_origins[curr_origin], intersectionOrderLock);
    }
  }

  lock_release(intersectionOrderLock);
}
