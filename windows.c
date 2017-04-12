#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <cairo.h>
#include <cairo-xlib.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "lidar.h"
#include "base_module.h"
#include "gui.h"

#define COMPASS_RADIUS 70

#define DISPLAY_FREQUENCY 10
#define CROP_DISTANCE (6000 * 4)
#define ENLARGE_CENTER 500

#define LOOP_DELAY 30000

#define RIGHT_ARROW 0xff53
#define LEFT_ARROW 0xff51
#define UP_ARROW 0xff52
#define DOWN_ARROW 0xff54
#define ESC_KEY 0xff1b
#define LEFT_MOUSE_BUTTON -1

#define RAY_USUAL_TYPE 1
#define RAY_AZIMUTH_TYPE 2
#define RAY_ZERO_TYPE 3

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

const int minx = 1, miny = 1, maxx = 8, maxy = 8;

static lidar_data_type lidar_data;

static double guiWidth = 600;
static double guiHeight = 600;

void draw_ray(int i, int ray_type)
{
    int x, y, center_x, center_y;
    if (ray_type == RAY_USUAL_TYPE)
    {
      x = (int)((ENLARGE_CENTER + lidar_data.distance[i]) / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      y = (int)((ENLARGE_CENTER + lidar_data.distance[i]) / 32000.0 * guiHeight * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
     center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
     center_y = (int)(ENLARGE_CENTER / 32000.0 * guiHeight * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.1, 0.1, 0.8);
    } else if (ray_type == RAY_AZIMUTH_TYPE)
    {
      x = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * sin(M_PI * i / 64.0 / 180.0) + guiWidth / 2);
      y = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * cos(M_PI * i / 64.0 / 180.0) + guiHeight / 2);
      center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * i / 64.0 / 180.0) + guiWidth / 2);
      center_y = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * cos(M_PI * i / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.8, 0.8, 0.3);
    } else if (ray_type == RAY_ZERO_TYPE)
    {
      x = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      y = (int)(CROP_DISTANCE / 32000.0 * guiWidth * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      center_x = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * sin(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiWidth / 2);
      center_y = (int)(ENLARGE_CENTER / 32000.0 * guiWidth * 0.45 * cos(M_PI * lidar_data.angle[i] / 64.0 / 180.0) + guiHeight / 2);
      cairo_set_source_rgb(gui, 0.8, 0.8, 0.8);
    }
    
    cairo_move_to(gui, x, guiHeight - y);
    cairo_line_to(gui, center_x, guiHeight - center_y);
    cairo_stroke(gui);
    cairo_set_source_rgb(gui, 1, 0.3, 0.3);
    cairo_arc(gui, x, guiHeight - y, 2, 0, 2 * M_PI);
    cairo_stroke(gui);
}

void *gui_thread(void *arg)
{
    double cpWidth = 350;
    double cpHeight = 200;
    base_data_type base_data;
    int disp_counter = 0;

    while (program_runs)
    {
      disp_counter++;
      if (disp_counter == DISPLAY_FREQUENCY)
      {
        disp_counter = 0;
        get_base_data(&base_data);
        get_lidar_data(&lidar_data);

        cairo_push_group(gui);
        cairo_set_source_rgb(gui, 1, 1, 1);
        cairo_paint(gui);
        cairo_set_line_width(gui, 2);

        for (int i = 0; i < lidar_data.count; i++)
        {
	  if (lidar_data.angle[i] > (120 * 64) && lidar_data.angle[i] < (240 * 64)) continue;
          if (lidar_data.distance[i] == 0) {
            draw_ray(i, RAY_ZERO_TYPE);
            continue;
          }
          if (lidar_data.distance[i] > CROP_DISTANCE) lidar_data.distance[i] = CROP_DISTANCE;
          draw_ray(i, RAY_USUAL_TYPE);
        }

        if (get_current_azimuth() != NO_AZIMUTH)
          draw_ray(get_current_azimuth() - base_data.heading, RAY_AZIMUTH_TYPE);

        cairo_pop_group_to_source(gui);
        cairo_paint(gui);
        cairo_surface_flush(gui_surface);
        //gui_cairo_check_event(gui_surface, 0);

        cairo_push_group(cp_gui);
        cairo_set_source_rgb(cp_gui, 1, 1, 1);
        cairo_paint(cp_gui);
        cairo_set_line_width(cp_gui, 3);
        cairo_set_source_rgb(cp_gui, 0.1, 0.3, 1);
        cairo_arc(cp_gui, cpWidth / 2, cpHeight / 2, COMPASS_RADIUS + 5, 0, 2 * M_PI);
        cairo_stroke(cp_gui);
        double compass_x = COMPASS_RADIUS * sin(M_PI * base_data.heading / 180.0);
        double compass_y = COMPASS_RADIUS * cos(M_PI * base_data.heading / 180.0);
        cairo_set_source_rgb(cp_gui, 1, 0, 0.2);
        cairo_move_to(cp_gui, cpWidth / 2 + compass_x, cpHeight / 2 - compass_y);
        cairo_line_to(cp_gui, cpWidth / 2, cpHeight / 2);
        cairo_stroke(cp_gui);
        cairo_pop_group_to_source(cp_gui);
        cairo_paint(cp_gui);
        cairo_surface_flush(cp_gui_surface);
      }

        int event = gui_cairo_check_event(cp_gui_surface, 0);
        if ((event != 0) && (!start_automatically))
          start_automatically = 1;
        switch (event)
        {
         case RIGHT_ARROW:
            user_dir = USER_DIR_RIGHT;
            user_control = 1;
            break;

         case LEFT_ARROW:
            user_dir = USER_DIR_LEFT;
            user_control = 1;
            break;

         case DOWN_ARROW:
            user_dir = USER_DIR_BACK;
            user_control = 1;
            break;

         case UP_ARROW:
            user_dir = USER_DIR_ONOFF;
            user_control = 1;
            break;

         case ESC_KEY:
            program_runs = 0;
            mikes_log(ML_INFO, "quit by ESC");
            break;

         case 32:
            user_dir = USER_DIR_BACKUP;
            user_control = 1;
            break;

         case LEFT_MOUSE_BUTTON:
            user_control = 1 - user_control;
            break;

         //default:
         //   printf("event %d\n", event);
        }

    usleep(LOOP_DELAY);
    }

    gui_shutdown();
    mikes_log(ML_INFO, "gui quits.");
    threads_running_add(-1);
    return 0;
}

