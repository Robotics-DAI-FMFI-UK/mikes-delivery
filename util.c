#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <libxml/xmlreader.h>
#include <sys/time.h>

#include "mikes_logs.h"
#include "util.h"

double distance(double x1, double y1, double x2, double y2)
{
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

long msec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000L * tv.tv_sec + tv.tv_usec / 1000L;
}

long usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000000L * tv.tv_sec + tv.tv_usec;
}

void say(char *sentence)
{
    time_t current_time;
    time(&current_time);
    char buf[128];
    sprintf(buf, "/bin/bash -c \"echo '%s' | espeak -v en-us -p 90 -a 400 2>/dev/null >/dev/null & \"", sentence);
    system(buf);
}

int parse_element(xmlNode* a_node, line *lines)
{
   xmlNode *cur_node = NULL;
   
   int i = 0;
   
   for (cur_node = a_node; cur_node; cur_node = cur_node->next)
   {
      if (cur_node->type == XML_ELEMENT_NODE)
      {
         if (xmlStrEqual(cur_node->name, (const xmlChar *) "line")
             && xmlGetProp(cur_node, (const xmlChar *) "lineId") != NULL)
         {
            xmlChar *x1 = xmlGetProp(cur_node, (const xmlChar *) "x1");
            xmlChar *y1 = xmlGetProp(cur_node, (const xmlChar *) "y1");
            xmlChar *x2 = xmlGetProp(cur_node, (const xmlChar *) "x2");
            xmlChar *y2 = xmlGetProp(cur_node, (const xmlChar *) "y2");
            xmlChar *lineId = xmlGetProp(cur_node, (const xmlChar *) "lineId");
            lines[i].x1 = (double) atoi((const char *) x1);
            lines[i].y1 = (double) atoi((const char *) y1);
            lines[i].x2 = (double) atoi((const char *) x2);
            lines[i].y2 = (double) atoi((const char *) y2);
            lines[i].x1 = (double) atoi((const char *) x1);
            lines[i].id = atoi((const char *) lineId);
            ++i;
            //printf("Line %.3d: x1 = %.4d y1 = %.4d x2 = %.4 y2 = %.4d\n",
            //		atoi((const char *) lineId), atoi((const char *) x1), atoi((const char *) y1), atoi((const char *) x2), atoi((const char *) y2));
         }
      }
   }
   return i;
}

int get_lines_from_file(const char *filename, line *lines)
{
   
   xmlDoc *doc = NULL;
   xmlNode *root_element = NULL;
   
   /*
    * this initialize the library and check potential ABI mismatches
    * between the version it was compiled for and the actual shared
    * library used.
    */
   LIBXML_TEST_VERSION
   
   /*parse the file and get the DOM */
   doc = xmlReadFile(filename, NULL, 0);
   
   if (doc == NULL) {
      mikes_log(ML_ERR, "SVG parse error: could not parse svg file");
   }
   
   /*Get the root element node */
   root_element = xmlDocGetRootElement(doc);
   
   int line_n = parse_element(root_element->children, lines);
   
   /* free the document */
   xmlFreeDoc(doc);
   /* Free the global variables that may have been allocated by the parser. */
   xmlCleanupParser();
   
   return line_n;
}

int get_polygons(point* p_i, point* p_a, point* p_h, line *lines)
{

   int i;
   const int j = NUMBER_OF_VERTICES_I;
   const int k = NUMBER_OF_VERTICES_I + NUMBER_OF_VERTICES_A;
   for (i = 0; i < j; ++i) {
	  p_i[i].x = lines[i].x1;
	  p_i[i].y = lines[i].y1;
   }
   for (i = j; i < k; ++i) {
	  p_a[i-j].x = lines[i].x1;
	  p_a[i-j].y = lines[i].y1;
   }
   for (i = k; i <= 162; ++i) {
	  p_h[i-k].x = lines[i].x1;
	  p_h[i-k].y = lines[i].y1;
   }
   
   return 1;
}

long usec_time()
{
   struct timeval tv;
   gettimeofday(&tv, 0);
   return 1000000L * tv.tv_sec + tv.tv_usec;
}

double normAlpha(double alpha){
   if(alpha < 0){
      while(alpha < 0)
         alpha += 360;
   }
   else
      while(alpha >= 360)
         alpha -= 360;
   return alpha;
}

double rad_normAlpha(double alpha){
   if(alpha < 0){
      while(alpha < 0)
         alpha += 2.0 * M_PI;
   }
   else
      while(alpha >= 2.0 * M_PI)
         alpha -= 2.0 * M_PI;
   return alpha;
}

