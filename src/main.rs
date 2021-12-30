use std::{
    fs::{File, OpenOptions},
    io::BufWriter,
    path::Path,
};

use stl_io::{Triangle, Vector, Vertex};

#[derive(Debug)]
struct Dimensions {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
}
const IMAGE_SIZE: usize = 500;

fn main() {
    println!("Start of Program");

    let path = Path::new(r"output.png");
    let mut data = [1; IMAGE_SIZE * IMAGE_SIZE];

    let mut file = OpenOptions::new().read(true).open("mesh.stl").unwrap();
    let obj = stl_io::create_stl_reader(&mut file).unwrap();
    //convert iter to an array
    let mut triangle_vec: Vec<Triangle> = Vec::new();
    for tri in obj {
        triangle_vec.push(tri.unwrap());
    }
    println!("number of triangles: {0}", triangle_vec.len());
    //Find the bounding box of the model
    let mut obj_dim = obj_dimensions(&triangle_vec);
    //Move object so its base is at z=0
    for triangle in &mut triangle_vec {
        (0..3).for_each(|point| {
            triangle.vertices[point] = Vertex::new([
                triangle.vertices[point][0] - obj_dim.x_min,
                triangle.vertices[point][1] - obj_dim.y_min,
                triangle.vertices[point][2] - obj_dim.z_min,
            ]);
        });
    }

    obj_dim = obj_dimensions(&triangle_vec);
    let plane_height: f32 = 13.6;

    for triangle in triangle_vec {
        let intersect: [bool; 3] = [
            triangle.vertices[0][2] >= plane_height,
            triangle.vertices[1][2] >= plane_height,
            triangle.vertices[2][2] >= plane_height,
        ];
        if intersect[0] as u8 + intersect[1] as u8 + intersect[2] as u8 == 1
            || intersect[0] as u8 + intersect[1] as u8 + intersect[2] as u8 == 2
        {
            //println!("tri intersects");

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
            let slopes: [Vector<f32>; 2] = [
                Vector::new([
                    others[0][0] - shared_point[0],
                    others[0][1] - shared_point[1],
                    others[0][2] - shared_point[2],
                ]),
                Vector::new([
                    others[1][0] - shared_point[0],
                    others[1][1] - shared_point[1],
                    others[1][2] - shared_point[2],
                ]),
            ];
            let transforms: [f32; 2] = [
                (plane_height - shared_point[2]) / slopes[0][2],
                (plane_height - shared_point[2]) / slopes[1][2],
            ];
            let intersection_points: [Vector<f32>; 2] = [
                Vector::new([
                    shared_point[0] + slopes[0][0] * transforms[0],
                    shared_point[1] + slopes[0][1] * transforms[0],
                    shared_point[2] + slopes[0][2] * transforms[0],
                ]),
                Vector::new([
                    shared_point[0] + slopes[1][0] * transforms[1],
                    shared_point[1] + slopes[1][1] * transforms[1],
                    shared_point[2] + slopes[1][2] * transforms[1],
                ]),
            ];
            println!(
                "Intersection at points {0:?} & {1:?}",
                intersection_points[0], intersection_points[1]
            );
            draw_line(&obj_dim, &mut data, intersection_points);
        } else {
           // println!("no {0:?}", triangle.vertices);
        }
    }
    write_array_to_file(path, data);
}

fn draw_line(obj_dim: &Dimensions, data: &mut [u8], intersection_points: [Vector<f32>; 2]) {
    let multiplier = {
        if obj_dim.x_max > obj_dim.y_max {
            obj_dim.x_max
        } else {
            obj_dim.y_max
        }
    };
    /*data[((intersection_points[0][0] * IMAGE_SIZE as f32 / multiplier) as usize)
        + (intersection_points[0][1] * IMAGE_SIZE as f32 / multiplier) as usize
            * IMAGE_SIZE as usize] = 255;
    data[((intersection_points[1][0] * IMAGE_SIZE as f32 / multiplier) as usize)
        + (intersection_points[1][1] * IMAGE_SIZE as f32 / multiplier) as usize
            * IMAGE_SIZE as usize] = 255;*/
    let line_vec = Vector::new([
        intersection_points[1][0] - intersection_points[0][0],
        intersection_points[1][1] - intersection_points[0][1],
        intersection_points[1][2] - intersection_points[0][2],
    ]);
    for step in 0..100 {
        data[(((intersection_points[0][0] + line_vec[0] * step as f32 / 100f32) * IMAGE_SIZE as f32
            / multiplier) as usize)
            + ((intersection_points[0][1] + line_vec[1] * step as f32 / 100f32) * IMAGE_SIZE as f32
                / multiplier) as usize
                * IMAGE_SIZE as usize] = 255;
    }
}

fn write_array_to_file(path: &Path, data: [u8; IMAGE_SIZE * IMAGE_SIZE]) {
    let file = File::create(path).unwrap();
    let w = &mut BufWriter::new(file);
    let mut encoder = png::Encoder::new(w, IMAGE_SIZE as u32, IMAGE_SIZE as u32);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();
    writer.write_image_data(&data).unwrap();
}

/// Return the axis coordinates of the models bounding box's six faces.
fn obj_dimensions(triangle_vec: &[Triangle]) -> Dimensions {
    let mut obj_dim = Dimensions {
        x_min: triangle_vec[0].vertices[0][0],
        x_max: triangle_vec[0].vertices[0][0],
        y_min: triangle_vec[0].vertices[0][1],
        y_max: triangle_vec[0].vertices[0][1],
        z_min: triangle_vec[0].vertices[0][2],
        z_max: triangle_vec[0].vertices[0][2],
    };
    triangle_vec.iter().for_each(|triangle| {
        (0..3).for_each(|point| {
            if triangle.vertices[point][0] < obj_dim.x_min {
                obj_dim.x_min = triangle.vertices[point][0]
            }
            if triangle.vertices[point][0] > obj_dim.x_max {
                obj_dim.x_max = triangle.vertices[point][0]
            }
            if triangle.vertices[point][1] < obj_dim.y_min {
                obj_dim.y_min = triangle.vertices[point][1]
            }
            if triangle.vertices[point][1] > obj_dim.y_max {
                obj_dim.y_max = triangle.vertices[point][1]
            }
            if triangle.vertices[point][2] < obj_dim.z_min {
                obj_dim.z_min = triangle.vertices[point][2]
            }
            if triangle.vertices[point][2] > obj_dim.z_max {
                obj_dim.z_max = triangle.vertices[point][2]
            }
        });
    });
    obj_dim
}
