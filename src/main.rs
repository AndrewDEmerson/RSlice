use std::{
    env,
    fs::{File, OpenOptions},
    io::BufWriter,
    path::Path,
};

use debug_print::debug_println;
use na::vector;

use crate::quadtree::PairedPoints;

extern crate nalgebra as na;

mod quadtree;

#[derive(Debug)]
struct Dimensions {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
}
const IMAGE_SIZE: usize = 600;

fn main() {
    println!("RSlice - written by Andrew Emerson 2021\n");
    // Read height to slice at from the cmd line
    let mut plane_height: f32 = 13.6;
    let mut stl_path = String::from("sphere.stl");
    let mut arg = env::args();
    arg.next();
    if let Some(v) = arg.next() {
        plane_height = v.parse::<f32>().unwrap();
    }
    if let Some(v) = arg.next() {
        stl_path = v;
    }
    debug_println!(
        "\tOperating on file: {file}\n\tSlicing on Z = {hght}",
        file = stl_path,
        hght = plane_height
    );

    let path = Path::new(r"output.png");
    let mut data = vec![0; IMAGE_SIZE * IMAGE_SIZE];

    let mut file = OpenOptions::new().read(true).open(&stl_path).unwrap_or_else(|error|{panic!("Could not open file: {0}\n\t{1:?}",&stl_path, error);});
    let obj = stl_io::create_stl_reader(&mut file).unwrap();
    // convert iter to an array
    let mut triangle_vec: Vec<NaTri> = Vec::new();
    for tri in obj {
        let read = tri.unwrap();
        let newtri = NaTri {
            normal: vector!(read.normal[0], read.normal[1], read.normal[2]),
            vertices: [
                na::point!(
                    read.vertices[0][0],
                    read.vertices[0][1],
                    read.vertices[0][2]
                ),
                na::point!(
                    read.vertices[1][0],
                    read.vertices[1][1],
                    read.vertices[1][2]
                ),
                na::point!(
                    read.vertices[2][0],
                    read.vertices[2][1],
                    read.vertices[2][2]
                ),
            ],
        };
        triangle_vec.push(newtri);
    }
    debug_println!("number of triangles: {0}", triangle_vec.len());
    //Find the bounding box of the model
    let mut obj_dim = obj_dimensions(&triangle_vec);
    //Move object so its base is at z=0 and in positive quadrant
    for triangle in &mut triangle_vec {
        (0..3).for_each(|point| {
            triangle.vertices[point].x -= obj_dim.x_min;
            triangle.vertices[point].y -= obj_dim.y_min;
            triangle.vertices[point].z -= obj_dim.z_min;
        });
    }
    obj_dim.x_max -= obj_dim.x_min;
    obj_dim.y_max -= obj_dim.y_min;
    obj_dim.z_max -= obj_dim.z_min;
    obj_dim.x_min = 0f32;
    obj_dim.y_min = 0f32;
    obj_dim.z_min = 0f32;

    let mut all_points: Vec<PairedPoints> = Vec::new();
    for triangle in triangle_vec {
        let intersect: [bool; 3] = [
            triangle.vertices[0].z >= plane_height,
            triangle.vertices[1].z >= plane_height,
            triangle.vertices[2].z >= plane_height,
        ];
        if intersect[0] as u8 + intersect[1] as u8 + intersect[2] as u8 == 1
            || intersect[0] as u8 + intersect[1] as u8 + intersect[2] as u8 == 2
        {
            // The triangle intersects the plane at this point
            // Determine the shared point (alone on one side of the plane), and the two other points that connect to it.
            let (shared_point, others) = {
                if intersect[0] != intersect[1] && intersect[0] != intersect[2] {
                    (
                        triangle.vertices[0],
                        [triangle.vertices[1], triangle.vertices[2]],
                    )
                } else if intersect[1] != intersect[0] && intersect[1] != intersect[2] {
                    (
                        triangle.vertices[1],
                        [triangle.vertices[0], triangle.vertices[2]],
                    )
                } else {
                    (
                        triangle.vertices[2],
                        [triangle.vertices[1], triangle.vertices[0]],
                    )
                }
            };
            // Calculate the two points where the triangle edges intersect with the plane,
            // the complete intersection is a line between these two points.
            let direction_vecs = [
                na::vector!(
                    others[0].x - shared_point[0],
                    others[0].y - shared_point[1],
                    others[0].z - shared_point[2]
                ),
                na::vector!(
                    others[1].x - shared_point[0],
                    others[1].y - shared_point[1],
                    others[1].z - shared_point[2]
                ),
            ];
            let transforms: [f32; 2] = [
                (plane_height - shared_point[2]) / direction_vecs[0].z,
                (plane_height - shared_point[2]) / direction_vecs[1].z,
            ];
            let intersection_points = [
                na::point!(
                    shared_point[0] + direction_vecs[0].x * transforms[0],
                    shared_point[1] + direction_vecs[0].y * transforms[0]
                ),
                na::point!(
                    shared_point[0] + direction_vecs[1].x * transforms[1],
                    shared_point[1] + direction_vecs[1].y * transforms[1]
                ),
            ];
            let tie = all_points.len();
            all_points.push(PairedPoints::new(intersection_points[0]));
            all_points.push(PairedPoints::new(intersection_points[1]));
            all_points[tie].partner_index = Some(tie + 1);
            all_points[tie].id = tie;
            all_points[tie + 1].partner_index = Some(tie);
            all_points[tie + 1].id = tie + 1;
            draw_line(&obj_dim, &mut data, intersection_points, 60);
        }
    }
    //write_array_to_file(path, &data);
    let mut pt_added: Vec<bool> = vec![false; all_points.len()];

    let mut qt = quadtree::QuadTree::new([
        na::point!(obj_dim.x_min, obj_dim.y_min),
        na::point!(obj_dim.x_max, obj_dim.y_max),
    ]);
    for pt in &all_points {
        qt.insert(pt);
    }
    let mut p1 = 0;
    let mut p2 = qt.get_neighboring_point(
        &all_points[all_points[p1].partner_index.unwrap()],
        &pt_added,
    );
    loop {
        pt_added[p2] = true;
        //println!("p1: {0}, p2: {1}", p1, p2);
        draw_line(
            &obj_dim,
            &mut data,
            [all_points[p1].coords, all_points[p2].coords],
            255,
        );
        p1 = p2;
        p2 = qt.get_neighboring_point(
            &all_points[all_points[p1].partner_index.unwrap()],
            &pt_added,
        );
        if p1 == 0 || p1 == p2 {
            break;
        }
    }
    write_array_to_file(path, &data);
}

fn draw_line(
    obj_dim: &Dimensions,
    data: &mut [u8],
    intersection_points: [na::Point2<f32>; 2],
    color: u8,
) {
    let multiplier = {
        if obj_dim.x_max > obj_dim.y_max {
            obj_dim.x_max + 1.0
        } else {
            obj_dim.y_max + 1.0
        }
    };
    let line_vec = na::vector!(
        intersection_points[1].x - intersection_points[0].x,
        intersection_points[1].y - intersection_points[0].y
    );
    // This is an inefficent way of drawing lines, fix it later
    for step in 0..1000 {
        data[(((intersection_points[0][0] + line_vec[0] * step as f32 / 1000f32)
            * IMAGE_SIZE as f32
            / multiplier) as usize)
            + ((intersection_points[0][1] + line_vec[1] * step as f32 / 1000f32)
                * IMAGE_SIZE as f32
                / multiplier) as usize
                * IMAGE_SIZE as usize] = color;
    }
}

// Take a pixel array and save it to the file
fn write_array_to_file(path: &Path, data: &[u8]) {
    let file = File::create(path).unwrap();
    let w = &mut BufWriter::new(file);
    let mut encoder = png::Encoder::new(w, IMAGE_SIZE as u32, IMAGE_SIZE as u32);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();
    writer.write_image_data(&data).unwrap();
}

/// Return the axis coordinates of the models bounding box's six faces.
fn obj_dimensions(triangle_vec: &[NaTri]) -> Dimensions {
    let mut obj_dim = Dimensions {
        x_min: triangle_vec[0].vertices[0].x,
        x_max: triangle_vec[0].vertices[0].x,
        y_min: triangle_vec[0].vertices[0].y,
        y_max: triangle_vec[0].vertices[0].y,
        z_min: triangle_vec[0].vertices[0].z,
        z_max: triangle_vec[0].vertices[0].z,
    };
    triangle_vec.iter().for_each(|triangle| {
        (0..3).for_each(|point| {
            if triangle.vertices[point].x < obj_dim.x_min {
                obj_dim.x_min = triangle.vertices[point].x;
            }
            if triangle.vertices[point].x > obj_dim.x_max {
                obj_dim.x_max = triangle.vertices[point].x;
            }
            if triangle.vertices[point].y < obj_dim.y_min {
                obj_dim.y_min = triangle.vertices[point].y;
            }
            if triangle.vertices[point].y > obj_dim.y_max {
                obj_dim.y_max = triangle.vertices[point].y;
            }
            if triangle.vertices[point].z < obj_dim.z_min {
                obj_dim.z_min = triangle.vertices[point].z;
            }
            if triangle.vertices[point].z > obj_dim.z_max {
                obj_dim.z_max = triangle.vertices[point].z;
            }
        });
    });
    obj_dim
}

#[derive(Clone, Debug, PartialEq)]
pub struct NaTri {
    /// Normal vector of the Triangle.
    pub normal: na::Vector3<f32>,
    /// The three vertices of the Triangle.
    pub vertices: [na::Point3<f32>; 3],
}

pub struct DirLine {
    // a perservation of the original triangle's normal, used to determine solidity of a polygon
    pub normal: na::Vector2<f32>,
    // Two end points of a line
    pub vertices: [na::Point2<f32>; 2],
}

pub struct Polygon {
    pub points: Vec<na::Point2<f32>>,
}
