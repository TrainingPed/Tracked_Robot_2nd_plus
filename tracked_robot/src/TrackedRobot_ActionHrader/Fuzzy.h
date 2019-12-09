void Fuzzy(double a, double b,double *output_l,double *output_r);	/*---For Mode1 Fuzzy---*/

void Fuzzy(double a, double b,double *output_l,double *output_r)

{
	//double result_l[21]={600,600,600,600,400,300,200,800,800,800,800,600,400,200,1000,1000,1000,1000,800,500,200};//1
	//double result_r[21]={200,300,400,600,600,600,600,200,400,600,800,800,800,800,200,500,800,1000,1000,1000,1000};
	//double result_l[21]={500,500,500,500,350,250,200,700,700,700,700,450,300,200,1000,1000,1000,1000,550,350,200};//2
	//double result_r[21]={200,250,350,500,500,500,500,200,300,450,700,700,700,700,200,350,550,1000,1000,1000,1000};
	//double result_l[21]={500,500,500,500,350,250,200,700,700,700,700,400,300,200,900,900,900,900,500,300,200};//3
	//double result_r[21]={200,250,350,500,500,500,500,200,300,400,700,700,700,700,200,300,500,900,900,900,900};
	double result_l[21]={500,500,500,500,300,250,200,750,750,750,750,400,300,200,950,950,950,950,450,300,200};//3-2
	double result_r[21]={200,250,300,500,500,500,500,200,300,400,750,750,750,750,200,300,450,950,950,950,950};
	//double result_l[21]={400,400,400,400,350,250,200,600,600,600,600,410,270,200,800,800,800,800,480,340,200};//4
	//double result_r[21]={200,250,350,400,400,400,400,200,270,410,600,600,600,600,200,340,480,800,800,800,800};
	//double result_l[21]={400,400,400,400,350,250,200,600,600,600,600,410,270,200,800,800,800,800,480,340,200};//5
	//double result_r[21]={200,250,350,400,400,400,400,200,270,410,600,600,600,600,200,340,480,800,800,800,800};


	double weight[21];
	double total_weight_l = 0,total_weight_r = 0;
	double total_product_l = 0,total_product_r = 0;
	double distance_near,distance_far,distance_veryfar;
	double pixel_l3,pixel_l2,pixel_l1,pixel_mid,pixel_r1,pixel_r2,pixel_r3;
	double nearzero;
	nearzero = 0.0;


	if(a >= 0 && a < 100)
	{
		distance_near = 1.0;
		distance_far = nearzero;
		distance_veryfar = nearzero;
	}
	else if(a >= 100&& a < 125)
	{
		distance_near = (175-a)/50;
		distance_far = 1.0-((175-a)/75);
		distance_veryfar = nearzero;
	}
	else if(a >= 125 && a <175)
	{
		distance_near = (175-a)/50;
		distance_far = 1.0-((175-a)/75) ;
		distance_veryfar = nearzero;
	}
	else if(a >= 175 && a <225)
	{
		distance_near = nearzero;
		distance_far = (250-a)/75;
		distance_veryfar = 1.0-((225-a)/50);
	}

	else if(a >= 225 && a <250)
	{
		distance_near = nearzero;
		distance_far = (250-a)/75;
		distance_veryfar = 1.0;
	}
	else if(a >= 250)
	{
		distance_near = nearzero;
		distance_far = nearzero;
		distance_veryfar = 1.0;
	}
////////////////////////////////////////////////////////////////////////
	if(b > 0.0 && b < 50)
	{
		pixel_l3 = 1.0;
		pixel_l2 = 1-((150-b)/150);
		pixel_l1 = nearzero;
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 50 && b < 100)
	{
		pixel_l3 = (200-b)/150;
		pixel_l2 = 1-((150-b)/150);
		pixel_l1 = nearzero;
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 100 && b < 150)
	{
		pixel_l3 = (200-b)/150;
		pixel_l2 = 1-((150-b)/150);
		pixel_l1 = 1-((250-b)/150);
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 150 && b < 200)
	{
		pixel_l3 = (200-b)/150;
		pixel_l2 = (300-b)/150;
		pixel_l1 = 1-((250-b)/150);
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 200 && b < 250)
	{
		pixel_l3 = nearzero;
		pixel_l2 = (300-b)/150;
		pixel_l1 = 1-((250-b)/150);
		pixel_mid = 1-((350-b)/150);
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 250 && b < 300)
	{
		pixel_l3 = nearzero;
		pixel_l2 = (300-b)/150;
		pixel_l1 = (400-b)/150;
		pixel_mid = 1-((350-b)/150);
		pixel_r1 = nearzero;
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 300 && b < 350)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = (400-b)/150;
		pixel_mid = 1-((350-b)/150);
		pixel_r1 = 1-((450-b)/150);
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 350 && b < 400)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = (400-b)/150;
		pixel_mid = (500-b)/150;
		pixel_r1 = 1-((450-b)/150);
		pixel_r2 = nearzero;
		pixel_r3 = nearzero;
	}
	else if(b >= 400 && b < 450)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid = (500-b)/150;
		pixel_r1 = 1-((450-b)/150);
		pixel_r2 = 1-((550-b)/150);
		pixel_r3 = nearzero;
	}
	else if(b >= 450 && b < 500)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid = (500-b)/150;
		pixel_r1 = (600-b)/150;
		pixel_r2 = 1-((550-b)/150);
		pixel_r3 = nearzero;
	}
	else if(b >= 500 && b < 550)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid =  nearzero;
		pixel_r1 = (600-b)/150;
		pixel_r2 = 1-((550-b)/150);
		pixel_r3 = 1-((650-b)/150);;
	}
	else if(b >= 550 && b < 600)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid = nearzero;
		pixel_r1 = (600-b)/150;
		pixel_r2 = (700-b)/150;
		pixel_r3 = 1-((650-b)/150);
	}
	else if(b >= 600 && b <650)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = (700-b)/150;
		pixel_r3 = 1-((650-b)/150);
	}
	else if(b >= 650)
	{
		pixel_l3 = nearzero;
		pixel_l2 = nearzero;
		pixel_l1 = nearzero;
		pixel_mid = nearzero;
		pixel_r1 = nearzero;
		pixel_r2 = (700-b)/150;
		pixel_r3 = 1.0;
	}


	weight[0] = distance_near*pixel_l3;
	weight[1] = distance_near*pixel_l2;
	weight[2] = distance_near*pixel_l1;
	weight[3] = distance_near*pixel_mid;
	weight[4] = distance_near*pixel_r1;
	weight[5] = distance_near*pixel_r2;
	weight[6] = distance_near*pixel_r3;

	weight[7] = distance_far*pixel_l3;
	weight[8] = distance_far*pixel_l2;
	weight[9] = distance_far*pixel_l1;
	weight[10] = distance_far*pixel_mid;
	weight[11] = distance_far*pixel_r1;
	weight[12] = distance_far*pixel_r2;
	weight[13] = distance_far*pixel_r3;

	weight[14] = distance_veryfar*pixel_l3;
	weight[15] = distance_veryfar*pixel_l2;
	weight[16] = distance_veryfar*pixel_l1;
	weight[17] = distance_veryfar*pixel_mid;
	weight[18] = distance_veryfar*pixel_r1;
	weight[19] = distance_veryfar*pixel_r2;
	weight[20] = distance_veryfar*pixel_l3;



	for(int i=0; i<=20; i++)
	{
		total_weight_l += weight[i];
		total_product_l += weight[i]*result_l[i];
	}
	*output_l = total_product_l/total_weight_l;

	for(int i=0; i<=20; i++)
	{
		total_weight_r += weight[i];
		total_product_r += weight[i]*result_r[i];
	}
	*output_r = total_product_r/total_weight_r;

}