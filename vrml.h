/* VRML 2.0 Reader 
 *
 * ver1.0 2005/09/27 Masaaki IIYAMA
 *
 */

typedef unsigned char uchar;

typedef struct {
  int w; /* image width */
  int h; /* image height */
  uchar *pix; /* pointer to the PPM image buffer */
  double *z; /* pointer to the z buffer */
} Image;

typedef struct {
  uchar  i[3]; /* Light Intensity in RGB */
  double d[3]; /* Light Direction */
  double p[3]; /* Light Source Location */
  int    light_type; /* 0:parallel, 1:point */
  int    color_type; /* 0:diffuse, 1:specular, 2:ambient */
} Light;

typedef struct {
  double p[3]; /* Camera Position */
  double d[3]; /* Camera Direction */
  double f;    /* Focus */
} Camera;

typedef struct {
  double diff[3]; /* Diffuse in RGB */
  double spec[3]; /* Specular in RGB */
  double ambi; /* Ambient */
  double shine;   /* Shiness */
} Surface;

typedef struct {
  double *vtx; /* Vertex List */
  int    *idx; /* Index List */
  int    vtx_num; /* number of vertice */
  int    idx_num; /* number of indices */
} Polygon;

int read_one_obj(
		 FILE *fp,
		 Polygon *poly,
		 Surface *surface);
