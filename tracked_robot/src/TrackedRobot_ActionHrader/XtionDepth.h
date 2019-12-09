/*---Dynamic Memory Allocation : Define---*/
#define row 640
#define col 480

/*---Give Coordinate to get Xtion(Depth) array data---*/
int depthMD(int x,int y);
/*---Tracked Robot Stair Depth Process---*/
void depth_find(int dc,int *mid_x2,int *mid_y2,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);

int depthMD(int x,int y)
{
    int idx=0;

    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)frameDepth.getData(); //Get Xrion array data

    idx = x+y * frameDepth.getWidth();  //Calculate the coordinates of the location

    return pDepth[idx];

}

void depth_find(int dc,int *mid_x2,int *mid_y2,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{
    /*--PS. " *XXXX " Define in mode X---*/
    int DMA_i,DMA_j;
    int mid_x=0,mid_y=0,c=0,cont=0,x=0,y=0,sw=0,i=0,j=0;

    //int ptr2[row][col] = {};

/*---Dynamic Memory Allocation : new ---*/
    int **ptr2=NULL;
	ptr2= new int*[row];
	for(DMA_i=0; DMA_i<row; DMA_i++)  ptr2[DMA_i]=new int[col];
    for(DMA_i=0; DMA_i<row; DMA_i++)
    {
        for(DMA_j=0; DMA_j<col; DMA_j++)  ptr2[DMA_i][DMA_j]=0;
    }

    if(dc==1) /*---For Mode 1---*/
    {

        for(i=0; i<640; i++) {
            for(j=1; j<480; j++) {
                if((abs(depthMD(i,j-1)-depthMD(i,j))>100)&&(abs(depthMD(i,j-1)-depthMD(i,j))<1000)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
                {
                    c=1;
                    ptr2[i][j]=c;

                }
            }
        }


        //Process depth data
        i=0;
        j=0;
        for(j=450;j>=35;j-=25)
        {
            for(i=5;i<=634;i+=5)
            {
                cont=0;x=0;
                if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
                {
                    cont++; x=i;sw=j;
                    while(i<635)
                    {
                        if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
                        {
                            if(ptr2[i+1][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+1][sw]==1) sw=sw;
                            else if(ptr2[i+1][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+1][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+1][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+1][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+1][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+1][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+1][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+1][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+1][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+1][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+1][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+1][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+1][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+1][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+1][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+1][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+1][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+1][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+1][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+1][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+1][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+1][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+1][sw-12]==1) sw=sw-12;
                            cont++; i++;
                        }
                        else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
                        {
                            if(ptr2[i+2][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+2][sw]==1) sw=sw;
                            else if(ptr2[i+2][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+2][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+2][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+2][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+2][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+2][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+2][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+2][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+2][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+2][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+2][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+2][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+2][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+2][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+2][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+2][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+2][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+2][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+2][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+2][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+2][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+2][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+2][sw-12]==1) sw=sw-12;
                            cont+=2; i+=2;
                        }
                        else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
                        {
                            if(ptr2[i+3][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+3][sw]==1) sw=sw;
                            else if(ptr2[i+3][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+3][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+3][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+3][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+3][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+3][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+3][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+3][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+3][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+3][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+3][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+3][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+3][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+3][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+3][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+3][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+3][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+3][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+3][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+3][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+3][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+3][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+3][sw-12]==1) sw=sw-12;
                            cont+=3; i+=3;
                        }
                        else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
                        {
                            if(ptr2[i+4][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+4][sw]==1) sw=sw;
                            else if(ptr2[i+4][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+4][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+4][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+4][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+4][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+4][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+4][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+4][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+4][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+4][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+4][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+4][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+4][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+4][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+4][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+4][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+4][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+4][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+4][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+4][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+4][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+4][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+4][sw-12]==1) sw=sw-12;
                            cont+=4; i+=4;
                        }
                        else	break ;
                    }
                }
                if(cont>200)//old=200
                {
                    mid_x=(x+i)/2;
                    mid_y=(j+sw)/2;

                    *midx=mid_x;
                    *midy=mid_y;

                    break;//inside for
                }
            }
            if(cont>200)//old=200
            {
                break; //outside for
            }
        }
        ////////////////////////////NO2///////////////////////////
        i=0;
        for(j=mid_y-30;j>=35;j-=25)
        {
            for(i=5;i<=634;i+=5)
            {
                cont=0;x=0;sw=0;
                if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
                {
                    cont++; x=i;sw=j;
                    while(i<635)
                    {
                        if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
                        {
                            if(ptr2[i+1][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+1][sw]==1) sw=sw;
                            else if(ptr2[i+1][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+1][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+1][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+1][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+1][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+1][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+1][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+1][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+1][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+1][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+1][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+1][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+1][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+1][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+1][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+1][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+1][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+1][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+1][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+1][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+1][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+1][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+1][sw-12]==1) sw=sw-12;
                            cont++; i++;
                        }
                        else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
                        {
                            if(ptr2[i+2][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+2][sw]==1) sw=sw;
                            else if(ptr2[i+2][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+2][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+2][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+2][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+2][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+2][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+2][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+2][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+2][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+2][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+2][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+2][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+2][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+2][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+2][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+2][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+2][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+2][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+2][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+2][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+2][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+2][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+2][sw-12]==1) sw=sw-12;
                            cont+=2; i+=2;
                        }
                        else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
                        {
                            if(ptr2[i+3][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+3][sw]==1) sw=sw;
                            else if(ptr2[i+3][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+3][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+3][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+3][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+3][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+3][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+3][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+3][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+3][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+3][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+3][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+3][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+3][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+3][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+3][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+3][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+3][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+3][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+3][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+3][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+3][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+3][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+3][sw-12]==1) sw=sw-12;
                            cont+=3; i+=3;
                        }
                        else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
                        {
                            if(ptr2[i+4][sw-1]==1) sw=sw-1;
                            else if(ptr2[i+4][sw]==1) sw=sw;
                            else if(ptr2[i+4][sw+1]==1) sw=sw+1;
                            else if(ptr2[i+4][sw+2]==1) sw=sw+2;
                            else if(ptr2[i+4][sw-2]==1) sw=sw-2;
                            else if(ptr2[i+4][sw+3]==1) sw=sw+3;
                            else if(ptr2[i+4][sw-3]==1) sw=sw-3;
                            else if(ptr2[i+4][sw+4]==1) sw=sw+4;
                            else if(ptr2[i+4][sw-4]==1) sw=sw-4;
                            else if(ptr2[i+4][sw+5]==1) sw=sw+5;
                            else if(ptr2[i+4][sw-5]==1) sw=sw-5;
                            else if(ptr2[i+4][sw+6]==1) sw=sw+6;
                            else if(ptr2[i+4][sw-6]==1) sw=sw-6;
                            else if(ptr2[i+4][sw+7]==1) sw=sw+7;
                            else if(ptr2[i+4][sw-7]==1) sw=sw-7;
                            else if(ptr2[i+4][sw+8]==1) sw=sw+8;
                            else if(ptr2[i+4][sw-8]==1) sw=sw-8;
                            else if(ptr2[i+4][sw+9]==1) sw=sw+9;
                            else if(ptr2[i+4][sw-9]==1) sw=sw-9;
                            else if(ptr2[i+4][sw+10]==1) sw=sw+10;
                            else if(ptr2[i+4][sw-10]==1) sw=sw-10;
                            else if(ptr2[i+4][sw+11]==1) sw=sw+11;
                            else if(ptr2[i+4][sw-11]==1) sw=sw-11;
                            else if(ptr2[i+4][sw+12]==1) sw=sw+12;
                            else if(ptr2[i+4][sw-12]==1) sw=sw-12;
                            cont+=4; i+=4;
                        }
                        else	break ;//out while
                    }
                }
                if(cont>200)//old=200
                {
                    *di=i;
                    *dj=j;
                    *dx=x;
                    *dsw=sw;

                    *mid_x2=(x+i)/2;
                    *mid_y2=(j+sw)/2;

                    break;//inside for
                }
            }
            if(cont>200)//old=200
            {
                break; //outside for
            }
        }

    }//if(dc=1) end

    else if(dc==21)	/*---For Mode 21---*/
	{

		for(int i=0;i<640;i++) {
			for(int j=1;j<480;j++) {
				//if((abs(depthMD(i,j-1)-depthMD(i,j))>200)&&(abs(depthMD(i,j-1)-depthMD(i,j))<350)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				if((abs(depthMD(i,j-1)-depthMD(i,j))>200)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))//Dec demo
				{
					c=1;
					ptr2[i][j]=c;

				}
			}
		}

		//Process depth data
		i=0;
		j=0;
		for(j=450;j>=35;j-=25)
		{
			for(i=5;i<=634;i+=5)
			{
				cont=0;x=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
				{
					cont++; x=i;sw=j;
					while(i<635)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							else if(ptr2[i+1][sw+3]==1) sw=sw+3;
							else if(ptr2[i+1][sw-3]==1) sw=sw-3;
							else if(ptr2[i+1][sw+4]==1) sw=sw+4;
							else if(ptr2[i+1][sw-4]==1) sw=sw-4;
							else if(ptr2[i+1][sw+5]==1) sw=sw+5;
							else if(ptr2[i+1][sw-5]==1) sw=sw-5;
							else if(ptr2[i+1][sw+6]==1) sw=sw+6;
							else if(ptr2[i+1][sw-6]==1) sw=sw-6;
							else if(ptr2[i+1][sw+7]==1) sw=sw+7;
							else if(ptr2[i+1][sw-7]==1) sw=sw-7;
							else if(ptr2[i+1][sw+8]==1) sw=sw+8;
							else if(ptr2[i+1][sw-8]==1) sw=sw-8;
							else if(ptr2[i+1][sw+9]==1) sw=sw+9;
							else if(ptr2[i+1][sw-9]==1) sw=sw-9;
							else if(ptr2[i+1][sw+10]==1) sw=sw+10;
							else if(ptr2[i+1][sw-10]==1) sw=sw-10;
							else if(ptr2[i+1][sw+11]==1) sw=sw+11;
							else if(ptr2[i+1][sw-11]==1) sw=sw-11;
							else if(ptr2[i+1][sw+12]==1) sw=sw+12;
							else if(ptr2[i+1][sw-12]==1) sw=sw-12;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							else if(ptr2[i+2][sw+3]==1) sw=sw+3;
							else if(ptr2[i+2][sw-3]==1) sw=sw-3;
							else if(ptr2[i+2][sw+4]==1) sw=sw+4;
							else if(ptr2[i+2][sw-4]==1) sw=sw-4;
							else if(ptr2[i+2][sw+5]==1) sw=sw+5;
							else if(ptr2[i+2][sw-5]==1) sw=sw-5;
							else if(ptr2[i+2][sw+6]==1) sw=sw+6;
							else if(ptr2[i+2][sw-6]==1) sw=sw-6;
							else if(ptr2[i+2][sw+7]==1) sw=sw+7;
							else if(ptr2[i+2][sw-7]==1) sw=sw-7;
							else if(ptr2[i+2][sw+8]==1) sw=sw+8;
							else if(ptr2[i+2][sw-8]==1) sw=sw-8;
							else if(ptr2[i+2][sw+9]==1) sw=sw+9;
							else if(ptr2[i+2][sw-9]==1) sw=sw-9;
							else if(ptr2[i+2][sw+10]==1) sw=sw+10;
							else if(ptr2[i+2][sw-10]==1) sw=sw-10;
							else if(ptr2[i+2][sw+11]==1) sw=sw+11;
							else if(ptr2[i+2][sw-11]==1) sw=sw-11;
							else if(ptr2[i+2][sw+12]==1) sw=sw+12;
							else if(ptr2[i+2][sw-12]==1) sw=sw-12;
							cont+=2; i+=2;
						}
						else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
						{
							if(ptr2[i+3][sw-1]==1) sw=sw-1;
							else if(ptr2[i+3][sw]==1) sw=sw;
							else if(ptr2[i+3][sw+1]==1) sw=sw+1;
							else if(ptr2[i+3][sw+2]==1) sw=sw+2;
							else if(ptr2[i+3][sw-2]==1) sw=sw-2;
							else if(ptr2[i+3][sw+3]==1) sw=sw+3;
							else if(ptr2[i+3][sw-3]==1) sw=sw-3;
							else if(ptr2[i+3][sw+4]==1) sw=sw+4;
							else if(ptr2[i+3][sw-4]==1) sw=sw-4;
							else if(ptr2[i+3][sw+5]==1) sw=sw+5;
							else if(ptr2[i+3][sw-5]==1) sw=sw-5;
							else if(ptr2[i+3][sw+6]==1) sw=sw+6;
							else if(ptr2[i+3][sw-6]==1) sw=sw-6;
							else if(ptr2[i+3][sw+7]==1) sw=sw+7;
							else if(ptr2[i+3][sw-7]==1) sw=sw-7;
							else if(ptr2[i+3][sw+8]==1) sw=sw+8;
							else if(ptr2[i+3][sw-8]==1) sw=sw-8;
							else if(ptr2[i+3][sw+9]==1) sw=sw+9;
							else if(ptr2[i+3][sw-9]==1) sw=sw-9;
							else if(ptr2[i+3][sw+10]==1) sw=sw+10;
							else if(ptr2[i+3][sw-10]==1) sw=sw-10;
							else if(ptr2[i+3][sw+11]==1) sw=sw+11;
							else if(ptr2[i+3][sw-11]==1) sw=sw-11;
							else if(ptr2[i+3][sw+12]==1) sw=sw+12;
							else if(ptr2[i+3][sw-12]==1) sw=sw-12;
							cont+=3; i+=3;
						}
						else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
						{
							if(ptr2[i+4][sw-1]==1) sw=sw-1;
							else if(ptr2[i+4][sw]==1) sw=sw;
							else if(ptr2[i+4][sw+1]==1) sw=sw+1;
							else if(ptr2[i+4][sw+2]==1) sw=sw+2;
							else if(ptr2[i+4][sw-2]==1) sw=sw-2;
							else if(ptr2[i+4][sw+3]==1) sw=sw+3;
							else if(ptr2[i+4][sw-3]==1) sw=sw-3;
							else if(ptr2[i+4][sw+4]==1) sw=sw+4;
							else if(ptr2[i+4][sw-4]==1) sw=sw-4;
							else if(ptr2[i+4][sw+5]==1) sw=sw+5;
							else if(ptr2[i+4][sw-5]==1) sw=sw-5;
							else if(ptr2[i+4][sw+6]==1) sw=sw+6;
							else if(ptr2[i+4][sw-6]==1) sw=sw-6;
							else if(ptr2[i+4][sw+7]==1) sw=sw+7;
							else if(ptr2[i+4][sw-7]==1) sw=sw-7;
							else if(ptr2[i+4][sw+8]==1) sw=sw+8;
							else if(ptr2[i+4][sw-8]==1) sw=sw-8;
							else if(ptr2[i+4][sw+9]==1) sw=sw+9;
							else if(ptr2[i+4][sw-9]==1) sw=sw-9;
							else if(ptr2[i+4][sw+10]==1) sw=sw+10;
							else if(ptr2[i+4][sw-10]==1) sw=sw-10;
							else if(ptr2[i+4][sw+11]==1) sw=sw+11;
							else if(ptr2[i+4][sw-11]==1) sw=sw-11;
							else if(ptr2[i+4][sw+12]==1) sw=sw+12;
							else if(ptr2[i+4][sw-12]==1) sw=sw-12;
							cont+=4; i+=4;
						}
						else
						{
                            break ;//out while
                        }
					}
				}
				//printf("cont=%d\n",cont);
				if(cont>200)//old=200
				{

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}
			if(cont>200)//old=200
			{
				break; //outside for
			}
		}
		////////////////////////////////NO2////////////////////////////
		i=0;
		for(j=mid_y-30;j>=35;j-=25)
		{
			for(i=5;i<=634;i+=5)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
				{
					cont++; x=i;sw=j;
					while(i<635)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							else if(ptr2[i+1][sw+3]==1) sw=sw+3;
							else if(ptr2[i+1][sw-3]==1) sw=sw-3;
							else if(ptr2[i+1][sw+4]==1) sw=sw+4;
							else if(ptr2[i+1][sw-4]==1) sw=sw-4;
							else if(ptr2[i+1][sw+5]==1) sw=sw+5;
							else if(ptr2[i+1][sw-5]==1) sw=sw-5;
							else if(ptr2[i+1][sw+6]==1) sw=sw+6;
							else if(ptr2[i+1][sw-6]==1) sw=sw-6;
							else if(ptr2[i+1][sw+7]==1) sw=sw+7;
							else if(ptr2[i+1][sw-7]==1) sw=sw-7;
							else if(ptr2[i+1][sw+8]==1) sw=sw+8;
							else if(ptr2[i+1][sw-8]==1) sw=sw-8;
							else if(ptr2[i+1][sw+9]==1) sw=sw+9;
							else if(ptr2[i+1][sw-9]==1) sw=sw-9;
							else if(ptr2[i+1][sw+10]==1) sw=sw+10;
							else if(ptr2[i+1][sw-10]==1) sw=sw-10;
							else if(ptr2[i+1][sw+11]==1) sw=sw+11;
							else if(ptr2[i+1][sw-11]==1) sw=sw-11;
							else if(ptr2[i+1][sw+12]==1) sw=sw+12;
							else if(ptr2[i+1][sw-12]==1) sw=sw-12;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							else if(ptr2[i+2][sw+3]==1) sw=sw+3;
							else if(ptr2[i+2][sw-3]==1) sw=sw-3;
							else if(ptr2[i+2][sw+4]==1) sw=sw+4;
							else if(ptr2[i+2][sw-4]==1) sw=sw-4;
							else if(ptr2[i+2][sw+5]==1) sw=sw+5;
							else if(ptr2[i+2][sw-5]==1) sw=sw-5;
							else if(ptr2[i+2][sw+6]==1) sw=sw+6;
							else if(ptr2[i+2][sw-6]==1) sw=sw-6;
							else if(ptr2[i+2][sw+7]==1) sw=sw+7;
							else if(ptr2[i+2][sw-7]==1) sw=sw-7;
							else if(ptr2[i+2][sw+8]==1) sw=sw+8;
							else if(ptr2[i+2][sw-8]==1) sw=sw-8;
							else if(ptr2[i+2][sw+9]==1) sw=sw+9;
							else if(ptr2[i+2][sw-9]==1) sw=sw-9;
							else if(ptr2[i+2][sw+10]==1) sw=sw+10;
							else if(ptr2[i+2][sw-10]==1) sw=sw-10;
							else if(ptr2[i+2][sw+11]==1) sw=sw+11;
							else if(ptr2[i+2][sw-11]==1) sw=sw-11;
							else if(ptr2[i+2][sw+12]==1) sw=sw+12;
							else if(ptr2[i+2][sw-12]==1) sw=sw-12;
							cont+=2; i+=2;
						}
						else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
						{
							if(ptr2[i+3][sw-1]==1) sw=sw-1;
							else if(ptr2[i+3][sw]==1) sw=sw;
							else if(ptr2[i+3][sw+1]==1) sw=sw+1;
							else if(ptr2[i+3][sw+2]==1) sw=sw+2;
							else if(ptr2[i+3][sw-2]==1) sw=sw-2;
							else if(ptr2[i+3][sw+3]==1) sw=sw+3;
							else if(ptr2[i+3][sw-3]==1) sw=sw-3;
							else if(ptr2[i+3][sw+4]==1) sw=sw+4;
							else if(ptr2[i+3][sw-4]==1) sw=sw-4;
							else if(ptr2[i+3][sw+5]==1) sw=sw+5;
							else if(ptr2[i+3][sw-5]==1) sw=sw-5;
							else if(ptr2[i+3][sw+6]==1) sw=sw+6;
							else if(ptr2[i+3][sw-6]==1) sw=sw-6;
							else if(ptr2[i+3][sw+7]==1) sw=sw+7;
							else if(ptr2[i+3][sw-7]==1) sw=sw-7;
							else if(ptr2[i+3][sw+8]==1) sw=sw+8;
							else if(ptr2[i+3][sw-8]==1) sw=sw-8;
							else if(ptr2[i+3][sw+9]==1) sw=sw+9;
							else if(ptr2[i+3][sw-9]==1) sw=sw-9;
							else if(ptr2[i+3][sw+10]==1) sw=sw+10;
							else if(ptr2[i+3][sw-10]==1) sw=sw-10;
							else if(ptr2[i+3][sw+11]==1) sw=sw+11;
							else if(ptr2[i+3][sw-11]==1) sw=sw-11;
							else if(ptr2[i+3][sw+12]==1) sw=sw+12;
							else if(ptr2[i+3][sw-12]==1) sw=sw-12;
							cont+=3; i+=3;
						}
						else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
						{
							if(ptr2[i+4][sw-1]==1) sw=sw-1;
							else if(ptr2[i+4][sw]==1) sw=sw;
							else if(ptr2[i+4][sw+1]==1) sw=sw+1;
							else if(ptr2[i+4][sw+2]==1) sw=sw+2;
							else if(ptr2[i+4][sw-2]==1) sw=sw-2;
							else if(ptr2[i+4][sw+3]==1) sw=sw+3;
							else if(ptr2[i+4][sw-3]==1) sw=sw-3;
							else if(ptr2[i+4][sw+4]==1) sw=sw+4;
							else if(ptr2[i+4][sw-4]==1) sw=sw-4;
							else if(ptr2[i+4][sw+5]==1) sw=sw+5;
							else if(ptr2[i+4][sw-5]==1) sw=sw-5;
							else if(ptr2[i+4][sw+6]==1) sw=sw+6;
							else if(ptr2[i+4][sw-6]==1) sw=sw-6;
							else if(ptr2[i+4][sw+7]==1) sw=sw+7;
							else if(ptr2[i+4][sw-7]==1) sw=sw-7;
							else if(ptr2[i+4][sw+8]==1) sw=sw+8;
							else if(ptr2[i+4][sw-8]==1) sw=sw-8;
							else if(ptr2[i+4][sw+9]==1) sw=sw+9;
							else if(ptr2[i+4][sw-9]==1) sw=sw-9;
							else if(ptr2[i+4][sw+10]==1) sw=sw+10;
							else if(ptr2[i+4][sw-10]==1) sw=sw-10;
							else if(ptr2[i+4][sw+11]==1) sw=sw+11;
							else if(ptr2[i+4][sw-11]==1) sw=sw-11;
							else if(ptr2[i+4][sw+12]==1) sw=sw+12;
							else if(ptr2[i+4][sw-12]==1) sw=sw-12;
							cont+=4; i+=4;
						}
						else	break ;//out while
					}
				}
				if(cont>150)//old=200
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					*mid_x2=(x+i)/2;
					*mid_y2=(j+sw)/2;

					break;//inside for
				}
			}
			if(cont>150)//old=200
			{
				break; //outside for
			}
		}

	}//else if(dc==21) end

    else if(dc==2)	/*---For Mode 2---*/
	{

		for(int i=0;i<640;i++) {
			for(int j=1;j<480;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>250)&&(abs(depthMD(i,j-1)-depthMD(i,j))<350)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;

				}
			}
		}


		i=0;
		j=0;
		for(j=475;j>=7;j-=5)
		{
			for(i=0;i<=637;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<637)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							cont+=2; i+=2;
						}

						else	break;
					}
				}
				if(cont>200)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;
				}

			}

			if(cont>200)    break; //outside for
		}

	}//	else if(dc==2) end

    else if(dc==4)	/*---For Mode 4---*/
	{
		for(i=0;i<640;i++) {
			for(j=1;j<480;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>200)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;
				}
			}
		}

		i=0;
		j=0;
		for(j=2;j<=477;j+=5)
		{
			for(i=0;i<=637;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<637)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
						if(ptr2[i+1][sw-1]==1) sw=sw-1;
						else if(ptr2[i+1][sw]==1) sw=sw;
						else if(ptr2[i+1][sw+1]==1) sw=sw+1;
						else if(ptr2[i+1][sw+2]==1) sw=sw+2;
						else if(ptr2[i+1][sw-2]==1) sw=sw-2;
						cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
						if(ptr2[i+2][sw-1]==1) sw=sw-1;
						else if(ptr2[i+2][sw]==1) sw=sw;
						else if(ptr2[i+2][sw+1]==1) sw=sw+1;
						else if(ptr2[i+2][sw+2]==1) sw=sw+2;
						else if(ptr2[i+2][sw-2]==1) sw=sw-2;
						cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}
				if(cont>200)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>200)    break; //outside for
		}

	}//	else if(dc==4) end

    else if(dc==5)	/*---For Mode 5---*/
	{
		for(i=0;i<640;i++) {
			for(j=1;j<480;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>100)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;
				}
			}
		}

		i=0;
		j=0;
		for(j=477;j>=2;j-=5)
		{
			for(i=0;i<=637;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<637)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}

				if(cont>200)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>200)    break; //outside for

		}

	}//	else if(dc==5) end

    else if(dc==7)	/*---For Mode 7---*/
	{
		for(i=0;i<640;i++) {
			for(j=1;j<480;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>100)&&(abs(depthMD(i,j-1)-depthMD(i,j))<300)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;
				}
			}
		}

		i=0;
		j=0;
		for(j=2;j<=477;j+=5)
		{
			for(i=0;i<=637;i+=3)
			{
				cont=0;x=0;sw=0;

				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<637)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}

				if(cont>200)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>200)    break; //outside for

		}

	}//	else if(dc==7) end

/*---Dynamic Memory Allocation : delete ---*/

	for(int DMA_i=0; DMA_i<row; DMA_i++)    delete ptr2[DMA_i];
	delete ptr2;

} //stairs_find end


