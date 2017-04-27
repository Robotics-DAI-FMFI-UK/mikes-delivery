#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <libxml/xmlreader.h>

#include "mikes_logs.h"
#include "mcl.h"

#define MAP_W 4578
#define MAP_H 4303
#define NUMBER_OF_VERTICES 1000
#define NUMBER_OF_VERTICES_I 101
#define NUMBER_OF_VERTICES_A 49
#define NUMBER_OF_VERTICES_H 13
#define ROOM_I 1
#define ROOM_ATRIUM 2
#define ROOM_H3_H6 3

pthread_mutex_t lidar_mcl_lock;

hypo_t hypo[2][HYPO_COUNT];
point poly_i[NUMBER_OF_VERTICES_I]; // pavilon I
point poly_a[NUMBER_OF_VERTICES_A]; // atrium
point poly_h[NUMBER_OF_VERTICES_H]; // miestnosti H3 a H6

int vert_n_i;
int vert_n_a;
int vert_n_h;

int activeHypo = 0;

int pnpoly(int n_vert, point *vertices, double test_x, double test_y)
{
    int i, j = 0;
    bool c = false;
    for (i = 0, j = n_vert - 1; i < n_vert; j = i++) {
        if ( ((vertices[i].y > test_y) != (vertices[j].y > test_y))
            && (test_x < (vertices[j].x - vertices[i].x) * (test_y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x) )
        {
            c = !c;
        }
    }
    return c;
}

int is_in_corridor(int cx, int cy) {
    if (pnpoly(vert_n_i, poly_i, cx, cy)
    && !pnpoly(vert_n_a, poly_a, cx, cy)
    && !pnpoly(vert_n_h, poly_h, cx, cy))
        return 1;
    else
        return 0;
}

static int parse_element(xmlNode* a_node, point* points, int room)
{
    xmlNode *cur_node = NULL;
    int start_line_id;
    int end_line_id;
    
    switch (room) {
        case ROOM_I:
            start_line_id = 0;
            end_line_id = 100;
            break;
        case ROOM_ATRIUM:
            start_line_id = 101;
            end_line_id = 149;
            break;
        case ROOM_H3_H6:
            start_line_id = 150;
            end_line_id = 162;
            break;
        default:
            mikes_log(ML_ERR, "Wrong ROOM_ while parsing svg map.");
            break;
    }
    
    int i = 0;
    
    for (cur_node = a_node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if (xmlStrEqual(cur_node->name, (const xmlChar *) "line")
                && xmlGetProp(cur_node, (const xmlChar *) "lineId") != NULL
                && atoi((const char *) xmlGetProp(cur_node, (const xmlChar *) "lineId")) >= start_line_id
                && atoi((const char *) xmlGetProp(cur_node, (const xmlChar *) "lineId")) <= end_line_id)
            {

                xmlChar *x1 = xmlGetProp(cur_node, (const xmlChar *) "x1");
                xmlChar *y1 = xmlGetProp(cur_node, (const xmlChar *) "y1");
                //xmlChar *x2 = xmlGetProp(cur_node, (const xmlChar *) "x2");
                //xmlChar *y2 = xmlGetProp(cur_node, (const xmlChar *) "y2");
                //xmlChar *lineId = xmlGetProp(cur_node, (const xmlChar *) "lineId");
                points[i].x = (double) atoi((const char *) x1);
                points[i].y = (double) atoi((const char *) y1);
                ++i;
                //printf("Line %i: x1 = %s y1 = %s x2 = %s y2 = %s\n", atoi((const char *) lineId), x1, y1, x2, y2);
            }
        }
    }
    return i;
}


int get_polygon_from_file(point *buffer, const char *filename, int room) {
    
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
        printf("error: could not parse file %s\n", filename);
    }
    
    /*Get the root element node */
    root_element = xmlDocGetRootElement(doc);
    
    int vert_n = parse_element(root_element->children, buffer, room);
    
    //printf("%i", vert_n);
    
//    for (int i = 0; i < num_of_vertices; ++i) {
//        printf("vertex (%i, %i)\n", buffer[i].x, buffer[i].y);
//    }
    
    /* free the document */
    xmlFreeDoc(doc);
    /* Free the global variables that may have been allocated by the parser. */
    xmlCleanupParser();
    
    return vert_n;
}

int init_mcl(){
    pthread_mutex_init(&lidar_mcl_lock, 0);
    
    // seed random generator
    time_t t;
    srand((unsigned) time(&t));
    
    // get vertices of polygons from file
    vert_n_i = get_polygon_from_file(poly_i, "mapa_pavilonu_I.svg", ROOM_I);
    vert_n_a = get_polygon_from_file(poly_a, "mapa_pavilonu_I.svg", ROOM_ATRIUM);
    vert_n_h = get_polygon_from_file(poly_h, "mapa_pavilonu_I.svg", ROOM_H3_H6);
    
    mikes_log_val(ML_INFO, "vert_n_i = ", vert_n_i);
    mikes_log_val(ML_INFO, "vert_n_a = ", vert_n_a);
    mikes_log_val(ML_INFO, "vert_n_h = ", vert_n_h);
    
    for (int i = 0; i < HYPO_COUNT; i++){
        
        double rand_x = 0;
        double rand_y = 0;
        
        do {
            rand_x = rand() % MAP_W;
            rand_y = rand() % MAP_H;
        } while (!pnpoly(vert_n_i, poly_i, rand_x, rand_y)
                 || pnpoly(vert_n_a, poly_a, rand_x, rand_y)
                 || pnpoly(vert_n_h, poly_h, rand_x, rand_y));
        
        
        hypo[0][i].x = hypo[1][i].x = rand_x;
        hypo[0][i].y = hypo[1][i].y = rand_y;
        hypo[0][i].alpha = hypo[1][i].alpha = rand() % 360;
        hypo[0][i].w = hypo[1][i].w = 0.3;
        
//        mikes_log_val(ML_INFO, "hypo id: ", i);
//        mikes_log_double2(ML_INFO, "hypo pos: ", hypo[0][i].x,hypo[0][i].y);
//        mikes_log_double2(ML_INFO, "hypo v&a: ", hypo[0][i].w,hypo[0][i].alpha);

        
    }
    return 0;
}

void get_mcl_data(hypo_t *buffer)
{
    pthread_mutex_lock(&lidar_mcl_lock);
    memcpy(buffer, hypo[activeHypo], sizeof(hypo_t) * HYPO_COUNT);
    pthread_mutex_unlock(&lidar_mcl_lock);
}
