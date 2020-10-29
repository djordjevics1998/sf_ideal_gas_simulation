#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define NOT_COLLIDING -1

#define COLL_A(v_1, v_2) pow((v_1 - v_2), 2)
#define COLL_B(k_1, v_1, k_2, v_2) ((k_1 - k_2) * (v_1 - v_2))

#define kB 0.0000000000000000000000138

typedef enum {
	type_gas,
	type_wall
} particle_type;

typedef struct {
	float m, r, x, y, z, v_x, v_y, v_z;
} Gas;

typedef struct {
	float x, y, z;
} Point;

typedef struct {
	float A, B, C, D;
} Plane;

typedef struct {
	Point points[3];
} Triangle;

typedef struct {
	float v[3];
} Vector;

Triangle newWall(Point* p1, Point* p2, Point* p3) {
	Triangle t;
	t.points[0] = *p1;
	t.points[1] = *p2;
	t.points[2] = *p3;
	return t;
}

Vector vectorFrom2Points(Point* p, Point* q) {
	Vector v;
	v.v[0] = p->x - q->x;
	v.v[1] = p->y - q->y;
	v.v[2] = p->z - q->z;
	return v;
}

Plane TriangleToPlane(Triangle* t) {
	Plane p;
	Vector a = vectorFrom2Points(&t->points[1], &t->points[0]),
		b = vectorFrom2Points(&t->points[2], &t->points[0]);
	p.A = a.v[1] * b.v[2] - a.v[2] * b.v[1];
	p.B = a.v[2] * b.v[0] - a.v[0] * b.v[2];
	p.C = a.v[0] * b.v[1] - a.v[1] * b.v[0];
	p.D = - (p.A * t->points[0].x + p.B * t->points[0].y + p.C * t->points[0].z);
	
	return p;
}

Plane copy(Plane* p) {
	Plane cp;
	cp.A = p->A;
	cp.B = p->B;
	cp.C = p->C;
	cp.D = p->D;
	return cp;
}

float collision(void* o1, particle_type type1, void* o2, particle_type type2) {
	if(type1 == type_wall && type2 == type_gas) return collision(o2, type2, o1, type1);
	else if(type1 != type_gas && type2 != type_gas) return NOT_COLLIDING;
	
	Gas* g1 = (Gas*) o1;
	float t1, t2;
	if(type1 == type_gas && type2 == type_gas) {
		Gas* g2 = (Gas*) o2;
		float a = COLL_A(g1->v_x, g2->v_x) + COLL_A(g1->v_y, g2->v_y) + COLL_A(g1->v_z, g2->v_z),
			b = 2 * (COLL_B(g1->x, g1->v_x, g2->x, g2->v_x) + COLL_B(g1->y, g1->v_y, g2->y, g2->v_y) + COLL_B(g1->z, g1->v_z, g2->z, g2->v_z)),
			c = COLL_A(g1->x, g2->x) + COLL_A(g1->y, g2->y) + COLL_A(g1->z, g2->z) - COLL_A(g1->r, -g2->r);
		if(b * b - 4 * a * c < 0) return NOT_COLLIDING;
		t1 = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;
		t2 = (-b + sqrt(b * b + 4 * a * c)) / 2 / a;
		if(t1 < 0 && t2 < 0) return NOT_COLLIDING;
		else return (t1 < 0) ? t2 : t1;
	} else {
		Triangle* t = (Triangle*) o2;
		Plane p = TriangleToPlane(t),
			p1 = copy(&p),
			p2 = copy(&p);
		float tmp = g1->r * sqrt(p.A * p.A  + p.B * p.B + p.C * p.C);
		p1.D += tmp;
		p2.D -= tmp;
		tmp = p.A * g1->v_x + p.B * g1->v_y + p.C * g1->v_z;
		t1 = - (p1.D + p1.A * g1->x + p1.B * g1->y + p1.C * g1->z) / tmp;
		t2 = - (p2.D + p2.A * g1->x + p2.B * g1->y + p2.C * g1->z) / tmp;
		if((t1 <= 0 && t2 <= 0) || t1 == 0 || t2 == 0) return NOT_COLLIDING;
		else if(t1 < 0) return t2;
		else if(t2 < 0) return t1;
		else return t1 < t2 ? t1 : t2;
	}
}

int main(int argc, char *argv[]) {
	int N, l;
	float T, tmp1, tmp2, r = 1000, a = 0.0001, m = 0.0001;
	scanf("%d %f", &N, &T);
	Gas gas[N];
	srand((unsigned int)time(NULL));
	for(l = 0; l < N; l++) {
		gas[l].x = ((float)(rand())/(float)(RAND_MAX) - 0.5f) * 2 * (r - 2 * a);
		gas[l].y = ((float)(rand())/(float)(RAND_MAX) - 0.5f) * 2 * (r - 2* a);
		gas[l].z = ((float)(rand())/(float)(RAND_MAX) - 0.5f) * 2 * (r - 2 * a);
		gas[l].r = a;
		gas[l].m = m;
		
		tmp1 = (float)(rand())/(float)(RAND_MAX);
		tmp2 = (float)(rand())/(float)(RAND_MAX);
		gas[l].v_x = sqrt(- 2 * log(tmp1) * kB * T / gas[l].m) * cos(2 * M_PI * tmp2);
		
		tmp1 = (float)(rand())/(float)(RAND_MAX);
		tmp2 = (float)(rand())/(float)(RAND_MAX);
		gas[l].v_y = sqrt(- 2 * log(tmp1) * kB * T / gas[l].m) * cos(2 * M_PI * tmp2);
		
		tmp1 = (float)(rand())/(float)(RAND_MAX);
		tmp2 = (float)(rand())/(float)(RAND_MAX);
		gas[l].v_z = sqrt(- 2 * log(tmp1) * kB * T / gas[l].m) * cos(2 * M_PI * tmp2);
	}
	
	Point temena[8];
	temena[0] = (Point) {.x = -r, .y = -r, .z = -r};
	temena[1] = (Point) {.x = r, .y = -r, .z = -r};
	temena[2] = (Point) {.x = -r, .y = r, .z = -r};
	temena[3] = (Point) {.x = r, .y = r, .z = -r};
	temena[4] = (Point) {.x = -r, .y = -r, .z = r};
	temena[5] = (Point) {.x = r, .y = -r, .z = r};
	temena[6] = (Point) {.x = -r, .y = r, .z = r};
	temena[7] = (Point) {.x = r, .y = r, .z = r};
	Triangle wall[12];
	wall[0] = newWall(&temena[0], &temena[1], &temena[2]);
	wall[1] = newWall(&temena[3], &temena[1], &temena[2]);
	wall[2] = newWall(&temena[4], &temena[5], &temena[6]);
	wall[3] = newWall(&temena[7], &temena[5], &temena[2]);
	wall[4] = newWall(&temena[0], &temena[2], &temena[4]);
	wall[5] = newWall(&temena[6], &temena[2], &temena[4]);
	wall[6] = newWall(&temena[1], &temena[3], &temena[5]);
	wall[7] = newWall(&temena[7], &temena[3], &temena[5]);
	wall[8] = newWall(&temena[0], &temena[1], &temena[4]);
	wall[9] = newWall(&temena[5], &temena[1], &temena[4]);
	wall[10] = newWall(&temena[2], &temena[3], &temena[6]);
	wall[11] = newWall(&temena[7], &temena[3], &temena[6]);
	
	
	/*gas[0].r = 5;
	gas[0].x = 15;
	gas[0].y = 0;
	gas[0].z = 0;
	gas[0].v_x = -1;
	gas[0].v_y = 0;
	gas[0].v_z = 0;
	
	gas[1].r = 5;
	gas[1].x = 40;
	gas[1].y = 0;
	gas[1].z = 0;
	gas[1].v_x = 0;
	gas[1].v_y = 0;
	gas[1].v_z = 0;
	
	Triangle t;
	t.points[0] = (Point) {.x = 10, .y = 0, .z = 0};
	t.points[1] = (Point) {.x = 10, .y = 10, .z = 10};
	t.points[2] = (Point) {.x = 10, .y = 0, .z = 10};*/
	
	//printf("%.2f", collision(&gas[0], type_gas, &gas[1], type_gas));
	//printf("%.2f", collision(&gas[0], type_gas, &t, type_wall));
	return 0;
}
