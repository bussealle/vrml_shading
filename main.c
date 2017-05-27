//
//  main.c
//  cg-kadai2
//
//  Created by bussealle on 2015/10/16.
//  Copyright (c) 2015年 Ryuhei. All rights reserved.
//


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <float.h>

#include "vrml.h"


//焦点距離
const double opt=256;
int optlong;

//光源方向
const double lightx=-1;
const double lighty=-1;
const double lightz=2;


//出力画像の大きさ
const int height=256;
const int width=256;
const int depth=255;

//ステージのRGB&Zバッファ
typedef struct{
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    double depth;
}rgb;

typedef struct{
    double x_max;
    double x_min;
    double y_max;
    double y_min;
    double z_min;
    double cent_x;
    double cent_y;
}center;


//プロトタイプ宣言
void **openArray(int,int,int);
void trans(Polygon *poly,int i,double xyz_new[][3]);
double xydiv(double xmax,double xmin,double ymax,double ymid,double ymin);
void nVector(Polygon *poly,int i,double nvect[3]);
void shading(Surface *surface,double nvect[3],double lightv[3],double evect[3],int col[3]);


/* VRML 2.0 Reader
 *
 * ver1.1 2005/10/06 Masaaki IIYAMA (bug fix)
 * ver1.0 2005/09/27 Masaaki IIYAMA
 *
 */


/*
 /////////////////////////////////////////////////////////////////
 */
#define MWS 256

static int strindex( char *s, char *t)
{
    int         i, j, k;
    
    for (i = 0; s[i] != '\0'; i++) {
        for (j = i, k = 0;  t[k] != '\0' && s[j] == t[k]; j++, k++) ;
        if (k > 0 && t[k] == '\0')
            return i;
    }
    return -1;
}

static int getword(
                   FILE *fp,
                   char word[],
                   int sl)
{
    int i,c;
    
    while ( (c = fgetc(fp)) != EOF && ( isspace(c) || c == '#')) {
        if ( c == '#' ) {
            while ( (c = fgetc(fp)) != EOF && c != '\n') ;
            if ( c == EOF ) return (0);
        }
    }
    if ( c == EOF )
        return (0);
    ungetc(c,fp);
    
    for ( i = 0 ; i < sl - 1 ; i++) {
        word[i] = fgetc(fp);
        if ( isspace(word[i]) )
            break;
    }
    word[i] = '\0';
    
    return i;
}

static int read_material(
                         FILE *fp,
                         Surface *surface,
                         char *b)
{
    while (getword(fp,b,MWS)>0) {
        if      (strindex(b,"}")>=0) break;
        else if (strindex(b,"diffuseColor") >= 0) {
            getword(fp,b,MWS);
            surface->diff[0] = atof(b);
            getword(fp,b,MWS);
            surface->diff[1] = atof(b);
            getword(fp,b,MWS);
            surface->diff[2] = atof(b);
        }
        else if (strindex(b,"ambientIntensity") >= 0) {
            getword(fp,b,MWS);
            surface->ambi = atof(b);
        }
        else if (strindex(b,"specularColor") >= 0) {
            getword(fp,b,MWS);
            surface->spec[0] = atof(b);
            getword(fp,b,MWS);
            surface->spec[1] = atof(b);
            getword(fp,b,MWS);
            surface->spec[2] = atof(b);
        }
        else if (strindex(b,"shininess") >= 0) {
            getword(fp,b,MWS);
            surface->shine = atof(b);
        }
    }
    return 1;
}

static int count_point(
                       FILE *fp,
                       char *b)
{
    int num=0;
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"[")>=0) break;
    }
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"]")>=0) break;
        else {
            num++;
        }
    }
    if ( num %3 != 0 ) {
        fprintf(stderr,"invalid file type[number of points mismatch]\n");
    }
    return num/3;
}

static int read_point(
                      FILE *fp,
                      Polygon *polygon,
                      char *b)
{
    int num=0;
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"[")>=0) break;
    }
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"]")>=0) break;
        else {
            polygon->vtx[num++] = atof(b);
        }
    }
    return num/3;
}

static int count_index(
                       FILE *fp,
                       char *b)
{
    int num=0;
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"[")>=0) break;
    }
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"]")>=0) break;
        else {
            num++;
        }
    }
    if ( num %4 != 0 ) {
        fprintf(stderr,"invalid file type[number of indices mismatch]\n");
    }
    return num/4;
}

static int read_index(
                      FILE *fp,
                      Polygon *polygon,
                      char *b)
{
    int num=0;
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"[")>=0) break;
    }
    while (getword(fp,b,MWS)>0) {
        if (strindex(b,"]")>=0) break;
        else {
            polygon->idx[num++] = atoi(b);
            if (num%3 == 0) getword(fp,b,MWS);
        }
    }
    return num/3;
}

int read_one_obj(
                 FILE *fp,
                 Polygon *poly,
                 Surface *surface)
{
    char b[MWS];
    int flag_material = 0;
    int flag_point = 0;
    int flag_index = 0;
    
    /* initialize surface */
    surface->diff[0] = 1.0;
    surface->diff[1] = 1.0;
    surface->diff[2] = 1.0;
    surface->spec[0] = 0.0;
    surface->spec[1] = 0.0;
    surface->spec[2] = 0.0;
    surface->ambi = 0.0;
    surface->shine = 0.2;
    
    if ( getword(fp,b,MWS) <= 0) return 0;
    
    poly->vtx_num = 0;
    poly->idx_num = 0;
    
    while (flag_material==0 || flag_point==0 || flag_index==0) {
        if      (strindex(b,"Material")>=0) {
            getword(fp,b,MWS);
            flag_material = 1;
        }
        else if (strindex(b,"point")>=0) {
            fprintf(stderr,"Counting... [point]\n");
            poly->vtx_num = count_point(fp, b);
            flag_point = 1;
        }
        else if (strindex(b,"coordIndex")>=0) {
            fprintf(stderr,"Counting... [coordIndex]\n");
            poly->idx_num = count_index(fp, b);
            flag_index = 1;
        }
        else if (getword(fp,b,MWS) <= 0) return 0;
    }
    
    flag_material = 0;
    flag_point = 0;
    flag_index = 0;
    
    fseek(fp, 0, SEEK_SET);
    poly->vtx = (double *)malloc(sizeof(double)*3*poly->vtx_num);
    poly->idx = (int *)malloc(sizeof(int)*3*poly->idx_num);
    while (flag_material==0 || flag_point==0 || flag_index==0) {
        if      (strindex(b,"Material")>=0) {
            fprintf(stderr,"Reading... [Material]\n");
            read_material(fp,surface,b);
            flag_material = 1;
        }
        else if (strindex(b,"point")>=0) {
            fprintf(stderr,"Reading... [point]\n");
            read_point(fp,poly,b);
            flag_point = 1;
        }
        else if (strindex(b,"coordIndex")>=0) {
            fprintf(stderr,"Reading... [coordIndex]\n");
            read_index(fp,poly,b);
            flag_index = 1;
        }
        else if (getword(fp,b,MWS) <= 0) return 0;
    }
    
    return 1;
}





//メイン
int main (int argc, char *argv[])
{
    FILE *fp;
    Polygon poly;
    Surface surface;
    
    char fname[200]; //ファイル名バッファ
    char foclen[100]; //焦点距離バッファ
    char *tok; //トークン
    int shad_sw=0; //シェーデイングスイッチ
    int cent_sw=0; //センタリングスイッチ
    
    
    
    optlong=opt; //デフォルト値を代入
    
    while(1){
        fprintf(stdout,"INPUT FILENAME\n");
        fgets(fname,sizeof(fname),stdin);
        
        tok = strtok(fname," -\n");
        
        if((fp = fopen(tok, "r"))==NULL){
            fprintf(stderr,"ERROR : CANNNOT OPEN FILE\n\n");
        }
        else{
            fprintf(stdout,">>OK! drawing \"%s\" ....\n\n\n",tok);
            
            while(1){
                
                tok = strtok(NULL," -\n");
                
                if (tok != NULL){
                    
                    if (strcmp(tok,"p")==0){ //phong
                        shad_sw=0;
                    }
                    else if (strcmp(tok,"g")==0){ //gourand
                        shad_sw=1;
                    }
                    else if (strcmp(tok,"c")==0){ //constant
                        shad_sw=2;
                    }
                    else if (strcmp(tok,"f")==0){ //focal length
                        fprintf(stdout,"INPUT FOCAL_LENGTH (default:256)\n");
                        fgets(foclen,sizeof(foclen),stdin);
                        if (atoi(foclen) != 0){
                            optlong=atoi(foclen);
                        }
                        else{
                            fprintf(stderr,"ERROR : NOT NUMBER CONTINUE with DEFAULT:256\n\n\n");
                        }
                    }
                    else if (strcmp(tok,"o")==0){ //centering
                        cent_sw=1;
                    }
                }
                else{
                    break;
                }
            }
            
            break;
        }
    }
    
   ///////////////////////////////////////////////////////////////
    
    read_one_obj(fp, &poly, &surface);
    
    fclose(fp);
    
    fprintf(stderr,"%d vertice are found.\n",poly.vtx_num);
    fprintf(stderr,"%d triangles are found.\n",poly.idx_num);
    
    /* i th vertex */
    for (int i = 0 ; i < poly.vtx_num ; i++ ) {
        fprintf(stdout,"%f %f %f # %d th vertex\n",
                poly.vtx[i*3+0], poly.vtx[i*3+1], poly.vtx[i*3+2],
                i);
    }
    
    /* i th triangle */
    for (int i = 0 ; i < poly.idx_num ; i++ ) {
        fprintf(stdout,"%d %d %d # %d th triangle\n",
                poly.idx[i*3+0], poly.idx[i*3+1], poly.idx[i*3+2],
                i);
    }
    
    /* material info */
    fprintf(stderr, "diffuseColor %f %f %f\n", surface.diff[0], surface.diff[1], surface.diff[2]);
    fprintf(stderr, "specularColor %f %f %f\n", surface.spec[0], surface.spec[1], surface.spec[2]);
    fprintf(stderr, "ambientIntensity %f\n", surface.ambi);
    fprintf(stderr, "shininess %f\n\n", surface.shine);
    switch ( shad_sw ){
        case 1:
            fprintf(stderr,">> Drawing with \"gouraud shading\"\n");
            break;
        case 2:
            fprintf(stderr,">> Drawing with \"constant shading\"\n");
            break;
        default:
            fprintf(stderr,">> Drawing with \"phong shading\"\n");
            break;
    }
    
    ///////////////////////////////////////////////////////////////
    
    
    double ave_n[poly.vtx_num][3]; //頂点ごとの接する平面の法線ベクトル平均
    double ave_n_temp[3];
    double c_temp;
    center cent;
    int ave_cnt=0;
    
    
    /* 図形のセンタリングのための計算 */
    cent.x_max=DBL_MIN;
    cent.x_min=DBL_MAX;
    cent.y_max=DBL_MIN;
    cent.y_min=DBL_MAX;

    for (int i=0; i < poly.vtx_num; i++) {
        c_temp = (poly.vtx[i*3+0]*(optlong))/poly.vtx[i*3+2];
        if (cent.x_max < c_temp){
            cent.x_max = c_temp;
        }
        if (cent.x_min > c_temp){
            cent.x_min = c_temp;
        }
        
        c_temp = (poly.vtx[i*3+1]*(optlong))/poly.vtx[i*3+2];
        if (cent.y_max < c_temp){
            cent.y_max = c_temp;
        }
        if (cent.y_min > c_temp){
            cent.y_min = c_temp;
        }
        
        for(int j=0; j < 3; j++){
            ave_n[i][j]=0; //初期化
        }
    }
    
    /* xy平面での物体の中心を求める*/
    cent.cent_x = (cent.x_max+cent.x_min)/2;
    cent.cent_y = (cent.y_max+cent.y_min)/2;
    //fprintf(stderr,"\n(%f,%f)\n",cent.cent_x,cent.cent_y);
    
    
    /* 頂点ごとの法線ベクトル */
    for (int i=0; i < poly.vtx_num; i++){
        for (int j=0; j < poly.idx_num; j++){
            for (int k=0; k < 3; k++){
                if ( i==poly.idx[j*3+k] ){
                    
                    nVector(&poly,j,ave_n_temp);
                    ave_n[i][0]=ave_n[i][0]+ave_n_temp[0];
                    ave_n[i][1]=ave_n[i][1]+ave_n_temp[1];
                    ave_n[i][2]=ave_n[i][2]+ave_n_temp[2];
                    ave_cnt++;
                    break;
                    
                }
            }
        }
        
        
        if ( ave_cnt > 0 ){
            ave_n[i][0]=ave_n[i][0]/ave_cnt;
            ave_n[i][1]=ave_n[i][1]/ave_cnt;
            ave_n[i][2]=ave_n[i][2]/ave_cnt;
            ave_cnt = 0;
        }
    }
    
    
    /* 出力バッファ領域確保 */
    rgb **buffer1;
    buffer1=(rgb**)openArray(height,width,sizeof(rgb));
    
    
    /* 出力バッファ初期化 */
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (j%16>=0 && j%16<8 && (i%16>=8 && i%16<16)){
                buffer1[i][j].red = 120;
                buffer1[i][j].green = 120;
                buffer1[i][j].blue = 120;
            }
            else if(j%16<16 && j%16>=8 && (i%16>=0 && i%16<8)){
                buffer1[i][j].red = 120;
                buffer1[i][j].green = 120;
                buffer1[i][j].blue = 120;
            }
            else{
                buffer1[i][j].red = 255;
                buffer1[i][j].green = 255;
                buffer1[i][j].blue = 255;
            }
            
            buffer1[i][j].depth = DBL_MAX;
        }
    }
    
    /* 光源方向数値代入 */
    double lightv[3];
    double lightleng;
    lightleng = sqrt(lightx*lightx+lighty*lighty+lightz*lightz);
    lightv[0]=lightx/lightleng;
    lightv[1]=lighty/lightleng;
    lightv[2]=lightz/lightleng;
    fprintf(stderr,">> lighting vector : (%d,%d,%d)\n\n",(int)lightx,(int)lighty,(int)lightz);
    
    
    double tri_new[3][3];//tri_new投影後の3点
    double cog[3]; //重心の座標
    double evect[3]; //ポリゴンの重心から視点に向かうベクトル
    double eleng=0.0;
    double nvect[3]; //法線ベクトル
    int col[3]; //RGB値
    double inv; //内積
    
    
    
    
    
    /* ポリゴン毎処理 */
    for (int k=0;k<poly.idx_num;k++){
        

        /* 法線単位ベクトル計算 */
        nVector(&poly,k,nvect);
        
        /* ポリゴンの重心から視点に向かう単位ベクトル */
        for (int i=0 ; i < 3 ; i++){
            cog[i]=(poly.vtx[poly.idx[k*3+0]*3+i]+poly.vtx[poly.idx[k*3+1]*3+i]+poly.vtx[poly.idx[k*3+2]*3+i])/3;
            eleng=eleng+cog[i]*cog[i];
        }
        eleng=sqrt(eleng);
        for (int i=0 ; i < 3 ; i++ ){
            evect[i]=-(cog[i]/eleng);
        }
        
        
        /* 隠面処理 */
        inv=nvect[0]*evect[0]+nvect[1]*evect[1]+nvect[2]*evect[2]; //ポリゴンの法線ベクトルと重心から視点に向かうベクトルとの内積
        if( inv > 0){
            
            int imax,imid,imin;//y座標ソート結果
            double Xd,Xd_x,Xd_z;//内分点とそれに対応する空間上の点のx,z座標
            
            
            /* 透視投影変換 */
            trans(&poly,k,tri_new);
            
            /* y座標ソート */
            if (tri_new[0][1]>=tri_new[1][1] && (tri_new[1][1]>=tri_new[2][1])){
                imax=0;imid=1;imin=2;
            }
            else if (tri_new[0][1]>=tri_new[2][1] && (tri_new[2][1]>=tri_new[1][1])){
                imax=0;imid=2;imin=1;
            }
            else if (tri_new[1][1]>=tri_new[2][1] && (tri_new[2][1]>=tri_new[0][1])){
                imax=1;imid=2;imin=0;
            }
            else if (tri_new[1][1]>=tri_new[0][1] && (tri_new[0][1]>=tri_new[2][1])){
                imax=1;imid=0;imin=2;
            }
            else if (tri_new[2][1]>=tri_new[1][1] && (tri_new[1][1]>=tri_new[0][1])){
                imax=2;imid=1;imin=0;
            }
            else if (tri_new[2][1]>=tri_new[0][1] && (tri_new[0][1]>=tri_new[1][1])){
                imax=2;imid=0;imin=1;
            }
            else{
                fprintf(stderr,"if error\n");
                exit(EXIT_FAILURE);
            }
            
            
            
            
            
            
            /* ラスタライズ */
            double x1,x2; //スキャンラインの両端x座標
            double x1_x,x2_x,x1_z,x2_z; //両端x座標に対応する３次元空間上の点のx,z座標
            double dz_dx; //３次元空間のdz/dx
            double tempz;
            double Xd_n[3],x1_n[3],x2_n[3],x1_x2_n[3];
            double Xd_col[3],x1_col[3],x2_col[3];
            int temp_col[3];
            int idx_col[3][3];
            int gap_x=width/2+(int)(-cent.cent_x+0.5)*cent_sw;
            int gap_y=width/2+(int)(-cent.cent_y+0.5)*cent_sw;
            
            
            /* 三角形ニ等分ラインのx座標計算 */
            Xd = xydiv(tri_new[imax][0],tri_new[imin][0],tri_new[imax][1],tri_new[imid][1],tri_new[imin][1]);
            Xd_x = xydiv(poly.vtx[poly.idx[k*3+imax]*3+0],poly.vtx[poly.idx[k*3+imin]*3+0],tri_new[imax][1],tri_new[imid][1],tri_new[imin][1]);
            Xd_z = xydiv(poly.vtx[poly.idx[k*3+imax]*3+2],poly.vtx[poly.idx[k*3+imin]*3+2],tri_new[imax][1],tri_new[imid][1],tri_new[imin][1]);
            
            
            /* 各シェーディングの前処理 */
            switch ( shad_sw ){
                case 1: //gourand
                    for (int i=0; i < 3; i++){
                        shading(&surface,ave_n[poly.idx[k*3+i]],lightv,evect,temp_col);
                        for (int j=0; j < 3; j++){
                            idx_col[i][j]=temp_col[j];
                        }
                    }
                    for (int i=0; i < 3; i++){
                        Xd_col[i] = xydiv(idx_col[imax][i],idx_col[imin][i],tri_new[imax][1],tri_new[imid][1],tri_new[imin][1]);
                    }
                    break;
                    
                case 2: //constant
                    shading(&surface,nvect,lightv,evect,col);
                    break;
                    
                default: //phong
                    for (int i=0; i < 3; i++){
                        Xd_n[i] = xydiv(ave_n[poly.idx[k*3+imax]][i],ave_n[poly.idx[k*3+imin]][i],tri_new[imax][1],tri_new[imid][1],tri_new[imin][1]);
                    }
                    break;
            }
            
           
            
            
            /* 上三角形ラスタライズ */
            
            for (int i = (int)(tri_new[imid][1]);i <= (int)(tri_new[imax][1]); i++){
                
                x1 = xydiv(tri_new[imax][0],Xd,tri_new[imax][1],i,tri_new[imid][1]);
                x1_x = xydiv(poly.vtx[poly.idx[k*3+imax]*3+0],Xd_x,tri_new[imax][1],i,tri_new[imid][1]);
                x1_z = xydiv(poly.vtx[poly.idx[k*3+imax]*3+2],Xd_z,tri_new[imax][1],i,tri_new[imid][1]);
                
                x2 = xydiv(tri_new[imax][0],tri_new[imid][0],tri_new[imax][1],i,tri_new[imid][1]);
                x2_x = xydiv(poly.vtx[poly.idx[k*3+imax]*3+0],poly.vtx[poly.idx[k*3+imid]*3+0],tri_new[imax][1],i,tri_new[imid][1]);
                x2_z = xydiv(poly.vtx[poly.idx[k*3+imax]*3+2],poly.vtx[poly.idx[k*3+imid]*3+2],tri_new[imax][1],i,tri_new[imid][1]);
                switch ( shad_sw ){
                    case 1: //gouraud
                        for (int j=0; j < 3; j++){
                            x1_col[j] = xydiv(idx_col[imax][j],Xd_col[j],tri_new[imax][1],i,tri_new[imid][1]);
                        }
                        for (int j=0; j < 3; j++){
                            x2_col[j] = xydiv(idx_col[imax][j],idx_col[imid][j],tri_new[imax][1],i,tri_new[imid][1]);
                        }
                        break;
                    case 2: //constant
                        //何もしない
                        break;
                    default: //phong
                        for (int j=0; j < 3; j++){
                            x1_n[j] = xydiv(ave_n[poly.idx[k*3+imax]][j],Xd_n[j],tri_new[imax][1],i,tri_new[imid][1]);
                        }
                        for (int j=0; j < 3; j++){
                            x2_n[j] = xydiv(ave_n[poly.idx[k*3+imax]][j],ave_n[poly.idx[k*3+imid]][j],tri_new[imax][1],i,tri_new[imid][1]);
                        }
                        break;
                    
                }
                /* 増分法のためのdz/dxの計算 */
                if ((x1_x-x2_x) != 0){
                    dz_dx=(x1_z-x2_z)/(x1_x-x2_x);
                }
                else{
                    dz_dx=0;
                }
                
                
                if (x1 > x2){
                    for (int j=(int)(x2);j<=(int)(x1);j++){
                        if (j+gap_x<height && (j+gap_x>=0 && (i+gap_y<width && i+gap_y>=0))){
                            tempz=x2_z+dz_dx*(j-(int)x2); //増分法によるZ座標の導出
                            switch ( shad_sw ) {
                                case 1: //gouraud
                                    for (int l=0; l < 3; l++){
                                        col[l] = (int)(xydiv(x1_col[l],x2_col[l],x1,j,x2)+0.5);
                                    }
                                    break;
                                case 2: //constant
                                    // 何もしない
                                    break;
                                    
                                default: //phong
                                    for (int l=0; l < 3; l++){
                                        x1_x2_n[l] = xydiv(x1_n[l],x2_n[l],x1,j,x2);
                                    }
                                    
                                    shading(&surface,x1_x2_n,lightv,evect,col);
                                    break;
                            }
                            
                            if(buffer1[i+gap_y][j+gap_x].depth > tempz){ //Zバッファを比較
                                buffer1[i+gap_y][j+gap_x].depth = tempz; //Zバッファを更新
                                buffer1[i+gap_y][j+gap_x].red=col[0];
                                buffer1[i+gap_y][j+gap_x].green=col[1];
                                buffer1[i+gap_y][j+gap_x].blue=col[2];
                            }
                        }
                    }
                }
                
                else{
                    for (int j=(int)(x1);j<=(int)(x2);j++){
                        if (j+gap_x<height && (j+gap_x>=0 && (i+gap_y<width && i+gap_y>=0))){
                            tempz=x1_z+dz_dx*(j-(int)x1);
                            switch ( shad_sw ) {
                                case 1: //gourand
                                    for (int l=0; l < 3; l++){
                                        col[l] = (int)(xydiv(x2_col[l],x1_col[l],x2,j,x1)+0.5);
                                    }
                                    break;
                                case 2: //constant
                                    // 何もしない
                                    break;
                                    
                                default: //phong
                                    for (int l=0; l < 3; l++){
                                        x1_x2_n[l] = xydiv(x2_n[l],x1_n[l],x2,j,x1);
                                    }
                                    
                                    shading(&surface,x1_x2_n,lightv,evect,col);
                                    break;
                            }
                            
                            
                            if(buffer1[i+gap_y][j+gap_x].depth > tempz){
                                buffer1[i+gap_y][j+gap_x].depth = tempz;
                                buffer1[i+gap_y][j+gap_x].red=col[0];
                                buffer1[i+gap_y][j+gap_x].green=col[1];
                                buffer1[i+gap_y][j+gap_x].blue=col[2];
                            }
                        }
                    }
                }
            }
            
            
            
            
            
            /* 下三角形ラスタライズ */
            
            for (int i=(int)(tri_new[imin][1]); i < (int)(tri_new[imid][1]); i++){
                
                x1 = xydiv(Xd,tri_new[imin][0],tri_new[imid][1],i,tri_new[imin][1]);
                x1_x = xydiv(Xd_x,poly.vtx[poly.idx[k*3+imin]*3+0],tri_new[imid][1],i,tri_new[imin][1]);
                x1_z = xydiv(Xd_z,poly.vtx[poly.idx[k*3+imin]*3+2],tri_new[imid][1],i,tri_new[imin][1]);
                
                
                x2 = xydiv(tri_new[imid][0],tri_new[imin][0],tri_new[imid][1],i,tri_new[imin][1]);
                x2_x = xydiv(poly.vtx[poly.idx[k*3+imid]*3+0],poly.vtx[poly.idx[k*3+imin]*3+0],tri_new[imid][1],i,tri_new[imin][1]);
                x2_z = xydiv(poly.vtx[poly.idx[k*3+imid]*3+2],poly.vtx[poly.idx[k*3+imin]*3+2],tri_new[imid][1],i,tri_new[imin][1]);
                
                
                switch ( shad_sw ){
                    case 1:
                        for (int j=0; j < 3; j++){
                            x1_col[j] = xydiv(Xd_col[j],idx_col[imin][j],tri_new[imid][1],i,tri_new[imin][1]);
                        }
                        for (int j=0; j < 3; j++){
                            x2_col[j] = xydiv(idx_col[imid][j],idx_col[imin][j],tri_new[imid][1],i,tri_new[imin][1]);
                        }
                        break;
                    case 2:
                        //何もしない
                        break;
                    default:
                        for (int j=0; j < 3; j++){
                            x1_n[j] = xydiv(Xd_n[j],ave_n[poly.idx[k*3+imin]][j],tri_new[imid][1],i,tri_new[imin][1]);
                        }
                        for (int j=0; j < 3; j++){
                            x2_n[j] = xydiv(ave_n[poly.idx[k*3+imid]][j],ave_n[poly.idx[k*3+imin]][j],tri_new[imid][1],i,tri_new[imin][1]);
                        }
                        break;
                        
                }
                
                /* 増分法のためのdz/dxの計算 */
                if((x1_x-x2_x) != 0){
                    dz_dx=(x1_z-x2_z)/(x1_x-x2_x);
                }
                else{
                    dz_dx=0;
                }
                
                if (x1 > x2){
                    for (int j=(int)(x2);j<=(int)(x1);j++){
                        if (j+gap_x<height && (j+gap_x>=0 && (i+gap_y<width && i+gap_y>=0))){
                            tempz=x2_z+dz_dx*(j-(int)x2);
                            switch ( shad_sw ) {
                                case 1: //gourand
                                    for (int l=0; l < 3; l++){
                                        col[l] = (int)(xydiv(x1_col[l],x2_col[l],x1,j,x2)+0.5);
                                    }
                                    break;
                                case 2: //constant
                                    // 何もしない
                                    break;
                                    
                                default: //phong
                                    for (int l=0; l < 3; l++){
                                        x1_x2_n[l] = xydiv(x1_n[l],x2_n[l],x1,j,x2);
                                    }
                                    
                                    shading(&surface,x1_x2_n,lightv,evect,col);
                                    break;
                            }
                            
                            if(buffer1[i+gap_y][j+gap_x].depth > tempz){
                                buffer1[i+gap_y][j+gap_x].depth = tempz;
                                buffer1[i+gap_y][j+gap_x].red=col[0];
                                buffer1[i+gap_y][j+gap_x].green=col[1];
                                buffer1[i+gap_y][j+gap_x].blue=col[2];
                            }
                        }
                    }
                }
                else{
                    for (int j=(int)(x1);j<=(int)(x2);j++){
                        
                        if (j+gap_x<height && (j+gap_x>=0 && (i+gap_y<width && i+gap_y>=0))){
                            tempz=x1_z+dz_dx*(j-(int)x1);
                            switch ( shad_sw ) {
                                case 1: //gourand
                                    for (int l=0; l < 3; l++){
                                        col[l] = (int)(xydiv(x2_col[l],x1_col[l],x2,j,x1)+0.5);
                                    }
                                    break;
                                case 2: //constant
                                    // 何もしない
                                    break;
                                    
                                default: //phong
                                    for (int l=0; l < 3; l++){
                                        x1_x2_n[l] = xydiv(x2_n[l],x1_n[l],x2,j,x1);
                                    }
                                    
                                    shading(&surface,x1_x2_n,lightv,evect,col);
                                    break;
                            }
                            
                            if(buffer1[i+gap_y][j+gap_x].depth > tempz){
                                buffer1[i+gap_y][j+gap_x].depth = tempz;
                                buffer1[i+gap_y][j+gap_x].red=col[0];
                                buffer1[i+gap_y][j+gap_x].green=col[1];
                                buffer1[i+gap_y][j+gap_x].blue=col[2];
                            }
                        }
                    }
                }
            }
            
        }
        
        
        
    }
    
    
    
    
    
    /* ファイル出力処理 */
    char ppm[]=".ppm";
    int tcnt=0;
    while (1){
        if(*(fname+tcnt)=='\0'){
            break;
        }
        else if(*(fname+tcnt)=='.'){
            fname[tcnt] = '\0';
            break;
        }
        else{
            tcnt++;
        }
    }
    strcat(fname,ppm);
    fp = fopen(fname,"wb");
    fprintf(fp,"P3\n%d %d\n%d\n",width,height,depth);
    
    char elem[10];
    
    for(int i=height-1;i>=0;i--){
        for(int j=width-1;j>=0;j--){
            sprintf(elem,"%d ",buffer1[i][j].red);
            fputs(elem,fp);
            sprintf(elem,"%d ",buffer1[i][j].green);
            fputs(elem,fp);
            sprintf(elem,"%d ",buffer1[i][j].blue);
            fputs(elem,fp);
        }
        fputs("\n",fp);
    }
    
    fputs("\n",fp);
    
    fclose(fp);
    free(buffer1);
    
    fprintf(stderr,"\n---------------------------\n!DONE! outputted \"%s\"\n",fname);
    
    return 0;
}


//シェーディング関数
void shading(Surface *surface,double nvect[3],double lightv[3],double evect[3],int col[3]){
    double Il[3];
    double inv;
    double cosv;
    
    /////* 1.環境光による反射強度 *////
    Il[0] = 1*surface->ambi;
    Il[1] = 1*surface->ambi;
    Il[2] = 1*surface->ambi;
    
    
    ////*  2.拡散反射光による反射強度*////
    inv=(lightv[0]*nvect[0]+lightv[1]*nvect[1]+lightv[2]*nvect[2]); //法線ベクトルと光源方向との内積
    cosv=inv/(1*sqrt(lightv[0]*lightv[0]+lightv[1]*lightv[1]+lightv[2]*lightv[2]));
    
    if ((-cosv)>=0){ //ポリゴンの表に光が当たっていれば
        Il[0] = Il[0]+1*surface->diff[0]*(-cosv);
        Il[1] = Il[1]+1*surface->diff[1]*(-cosv);
        Il[2] = Il[2]+1*surface->diff[2]*(-cosv);
    }
    
    
    ////* 3.鏡面反射光による反射強度 *////
    double svect[3]; //環境
    double e_i_vect[3]; //eとiの内積
    double esize=0.0; //eの大きさ
    double s_n_inv; //sとnの内積
    double temp_p=0.0;
    
    
    for (int j=0;j<3;j++){
        e_i_vect[j]=evect[j]-lightv[j];
        esize=esize+e_i_vect[j]*e_i_vect[j];
    }
    esize=sqrt(esize);
    for (int j=0;j<3;j++){
        svect[j]=e_i_vect[j]/esize;
    }
    
    s_n_inv=svect[0]*nvect[0]+svect[1]*nvect[1]+svect[2]*nvect[2];
    
    if (s_n_inv > 0){
        
        if (surface->shine > 1.0){
            surface->shine=1.0;
        }
        temp_p=pow(s_n_inv,128*surface->shine); //反射強度乗
        Il[0]=Il[0]+temp_p*surface->spec[0]*1;
        Il[1]=Il[1]+temp_p*surface->spec[1]*1;
        Il[2]=Il[2]+temp_p*surface->spec[2]*1;
    }
    
    
    ////* 塗りつぶすRGB値を代入 *////
    for (int i=0;i<3;i++){
        double temp;
        temp=255*Il[i]+0.5;
        if(temp>255.0){
            temp=255.0;
        }
        col[i]=(int)temp;
    }
    
}

//法線ベクトル計算
void nVector(Polygon *poly,int i,double nvect[3]){
    int vernum[3];
    vernum[0]=poly->idx[i*3+0];
    vernum[1]=poly->idx[i*3+1];
    vernum[2]=poly->idx[i*3+2];
    //座標→ベクトル 計算
    double vA[3],vB[3];
    for (int j=0;j<3;j++){
        vA[j]=poly->vtx[vernum[1]*3+j]-poly->vtx[vernum[0]*3+j];
        vB[j]=poly->vtx[vernum[2]*3+j]-poly->vtx[vernum[0]*3+j];
    }
    double nSize;
    nvect[0]=(vA[1]*vB[2]-vA[2]*vB[1]);
    nvect[1]=(vA[2]*vB[0]-vA[0]*vB[2]);
    nvect[2]=(vA[0]*vB[1]-vA[1]*vB[0]);
    nSize=sqrt(nvect[0]*nvect[0]+nvect[1]*nvect[1]+nvect[2]*nvect[2]);
    for (int j=0;j<3;j++){
        nvect[j]=nvect[j]/nSize;
    }
}

//相似比計算
double xydiv(double xmax,double xmin,double ymax,double ymid,double ymin){
    double temp1,temp2,xm;
    temp1=ymid-ymin;
    temp2=ymax-ymid;
    
    /* 外分防止 */
    if(temp1<0){
        temp1=0;
    }
    if(temp2<0){
        temp2=0;
    }
    
    if ((temp1+temp2)==0.0){
        xm=xmax;
    }
    else{
        xm=(xmax*temp1+xmin*temp2)/(temp1+temp2);
    }
    
    return xm;
}

//透視投影変換
void trans(Polygon *poly,int i,double xyz_new[][3]){
    double x,y,z;
    int vernum[3];
    
    vernum[0]=poly->idx[i*3+0];
    vernum[1]=poly->idx[i*3+1];
    vernum[2]=poly->idx[i*3+2];
    
    for (int j=0;j<3;j++){
        x=poly->vtx[vernum[j]*3+0];
        y=poly->vtx[vernum[j]*3+1];
        z=poly->vtx[vernum[j]*3+2];
        
        xyz_new[j][0]=(x*optlong)/z+0.3;
        xyz_new[j][1]=(y*optlong)/z+0.3;
        xyz_new[j][2]=z;
    }
}


//二次元配列領域確保
void **openArray(int row,int column,int sizeofVariable)
{
    int i,j;
    void **array=malloc(sizeof(void *)*row);
    if (array==NULL) {
        printf("** error ----  out of memory  **\n");
        exit(1);
    }
    for (i=0;i<row;i++) {
        array[i]=malloc(sizeofVariable*column);
        if (array[i]==NULL) {
            printf("** error ----  out of memory  **\n");
            exit(1);
        }
        for (j=0; j<sizeofVariable*column; j++) ((char *)array[i])[j]=0;
    }
    return array;
}


