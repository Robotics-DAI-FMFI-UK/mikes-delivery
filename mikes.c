#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "mikes.h"
#include "public_relations.h"
#include "lidar.h"
#include "mikes_logs.h"
#include "steering.h"
#include "gui.h"
#include "config_mikes.h"
#include "base_module.h"
#include "ncurses_control.h"
#include "mcl.h"
#include "pose.h"
#include "planner.h"
#include "util.h"
#include "tim571.h"

volatile unsigned char program_runs;
static pthread_mutex_t mikes_lock;
volatile unsigned short threads_running;
volatile unsigned char user_control;
volatile unsigned char user_dir;
volatile unsigned char start_automatically;

void threads_running_add(short x)
{
  pthread_mutex_lock(&mikes_lock);
  threads_running += x;
  pthread_mutex_unlock(&mikes_lock);
}

void signal_term_handler(int signum)
{
  program_runs = 0;
}

void say_greeting()
{
  say("Hello");
  sleep(1);
  say("my name is");
  sleep(1);
  say("me cash.");
  sleep(1);
  say("How do you do?");
}

int main(int argc, char **argv)
{
  program_runs = 1;
  threads_running = 1;
  pthread_mutex_init(&mikes_lock, 0);
  signal(SIGTERM, signal_term_handler);

  load_config();

  if ((!mikes_config.autostart) && (argc > 1))
    if (strcmp(argv[1], "autostart") == 0) return 0;

  init_mikes_logs();
  say_greeting();
  init_public_relations();
  init_pose(1, MAP_H);
  init_base_module();
  init_astar();
  init_lidar();
  init_tim571();
  init_mcl();
  init_planner();
  init_steering();
  init_ncurses_control();
  
  init_gui();

  while (program_runs)
  {
     sleep(1);
     test_tim571();
  }

  int old_tr = threads_running + 1;
  while (threads_running > 1)
  {
    usleep(10000);
    if (threads_running < old_tr)
    {
      char tr[50];
      sprintf(tr, "%d threads running", threads_running);
      mikes_log(ML_INFO, tr);
      old_tr = threads_running;
    }
  }

  mikes_log(ML_INFO, "Kocur mikes quits.");
  usleep(100000);
  return 0;
}

