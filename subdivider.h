#pragma once
#include<iostream>
#include <vector>
#include "fast_math.h"

using namespace std;

class Vertex {
public:
	vec3 position;
	vec3 normal;
	vec2 u,v;
};

class Element {
public:
	vec3 position;
	vec3 velocity;
	vec3 acceleration;
	//other parameters to be implemented by actuator needs
	float elasticity;
	float mass;
};

class Face {
public:
	vector<int> indices; //this points to the vertex list to retrieve vertices
	Element* e; //physic element contained in this face
};

class Mesh {
public:
	vector<Vertex*> vertices;
	vector<Face*> faces;
};

void normalize(vec3 r) {
	float n = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
	r[0] = r[0] / n;
	r[1] = r[1] / n;
	r[2] = r[2] / n;
};

class subdivider{

	void subdivide(Mesh& out, Mesh& in,int sub_time) {
	
		int flag = 0;
		out.faces = vector<Face*>(in.faces.size() * 4);//define size

		for (auto face : out.faces) {
				face->indices = vector<int>(4);
		};

		for (auto face : in.faces) {


			//read four verticies of each face
			vec3& ver1 = in.vertices[face->indices[0]]->position;
			vec3& ver2 = in.vertices[face->indices[1]]->position;
			vec3& ver3 = in.vertices[face->indices[2]]->position;
			vec3& ver4 = in.vertices[face->indices[3]]->position;

			//read four norms of each face
			vec3& norm1 = in.vertices[face->indices[0]]->normal;
			vec3& norm2 = in.vertices[face->indices[1]]->normal;
			vec3& norm3 = in.vertices[face->indices[2]]->normal;
			vec3& norm4 = in.vertices[face->indices[3]]->normal;

			normalize(norm1);
			normalize(norm2);
			normalize(norm3);
			normalize(norm4);

			vec3 p, p1, p2, p3, p4, p5, p6, p7, p8, n1, n2, n3, n4, n;

			vec3_sub(p1, ver2, ver1); //calculate midpoint of parallel line
			vec3_div(p1, p1, 2);
			vec3_sub(p3, ver4, ver3);
			vec3_div(p3, p2, 2);
			vec3_sub(p2, ver3, ver2);
			vec3_div(p2, p2, 2);
			vec3_sub(p4, ver1, ver4);
			vec3_div(p4, p4, 2);

			vec3_sub(p, p3, p1);
			vec3_div(p, p, 2);//centroid of original square

			vec3_sub(p5, p, ver1);//centroid of each small square
			vec3_div(p5, p5, 2);
			vec3_sub(p6, p, ver2);
			vec3_div(p6, p6, 2);
			vec3_sub(p7, p, ver3);
			vec3_div(p7, p7, 2);
			vec3_sub(p8, p, ver4);
			vec3_div(p8, p8, 2);

			vec3_add(n1, norm2, norm1); //calculate norm of new points
			vec3_div(n1, n1, 2);
			vec3_add(n2, norm3, norm2);
			vec3_div(n2, n2, 2);
			vec3_add(n3, norm4, norm3);
			vec3_div(n3, n3, 2);
			vec3_add(n4, norm1, norm4);
			vec3_div(n4, n4, 2);
			vec3_add(n, n1, n2);
			vec3_div(n, n, 2);

			normalize(n1);
			normalize(n2);
			normalize(n3);
			normalize(n4);
			normalize(n);

			int m = flag;

			*out.vertices[out.faces[m]->indices[0]]->position = *ver1;    //out vertex
			*out.vertices[out.faces[m]->indices[1]]->position = *p1;
			*out.vertices[out.faces[m]->indices[2]]->position = *p;
			*out.vertices[out.faces[m]->indices[3]]->position = *p4;

			*out.vertices[out.faces[m + 1]->indices[0]]->position = *p1;
			*out.vertices[out.faces[m + 1]->indices[1]]->position = *ver2;
			*out.vertices[out.faces[m + 1]->indices[2]]->position = *p2;
			*out.vertices[out.faces[m + 1]->indices[3]]->position = *p;

			*out.vertices[out.faces[m + 2]->indices[0]]->position = *p;
			*out.vertices[out.faces[m + 2]->indices[1]]->position = *p2;
			*out.vertices[out.faces[m + 2]->indices[2]]->position = *ver3;
			*out.vertices[out.faces[m + 2]->indices[3]]->position = *p3;

			*out.vertices[out.faces[m + 3]->indices[0]]->position = *p4;
			*out.vertices[out.faces[m + 3]->indices[1]]->position = *p;
			*out.vertices[out.faces[m + 3]->indices[2]]->position = *p3;
			*out.vertices[out.faces[m + 3]->indices[3]]->position = *ver4;

			*out.vertices[out.faces[m]->indices[0]]->normal = *norm1;   //out normal
			*out.vertices[out.faces[m]->indices[1]]->normal = *n1;
			*out.vertices[out.faces[m]->indices[2]]->normal = *n;
			*out.vertices[out.faces[m]->indices[3]]->normal = *n4;
			
			*out.vertices[out.faces[m+1]->indices[0]]->normal = *n1;
			*out.vertices[out.faces[m + 1]->indices[1]]->normal = *norm2;
			*out.vertices[out.faces[m + 1]->indices[2]]->normal = *n2;
			*out.vertices[out.faces[m + 1]->indices[3]]->normal = *n;

			*out.vertices[out.faces[m + 2]->indices[0]]->normal = *n;
			*out.vertices[out.faces[m + 2]->indices[1]]->normal = *n2;
			*out.vertices[out.faces[m + 2]->indices[2]]->normal = *norm3;
			*out.vertices[out.faces[m + 2]->indices[3]]->normal = *n3;

			*out.vertices[out.faces[m + 3]->indices[0]]->normal = *n4;
			*out.vertices[out.faces[m + 3]->indices[1]]->normal = *n;
			*out.vertices[out.faces[m + 3]->indices[2]]->normal = *n3;
			*out.vertices[out.faces[m + 3]->indices[3]]->normal = *norm4;

			*out.faces[m]->e->position = *p5;     //centroid of each out.faces
			*out.faces[m + 1]->e->position = *p6;
			*out.faces[m + 2]->e->position = *p7;
			*out.faces[m + 3]->e->position = *p8;

			flag = m + 4;
		};
	};
	public:
	void sd(Mesh& out, Mesh& in, int sub_time) {
		if (sub_time != 1) {
			Mesh o;
			o = in; 
			for (sub_time; sub_time != 1; sub_time=sub_time-1) {
				in = o;
				subdivide(out, in, sub_time);
                o = out;
			};
		};
		subdivide(out, in, sub_time);
	};
};


