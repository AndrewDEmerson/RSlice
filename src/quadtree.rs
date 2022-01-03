extern crate nalgebra as na;

#[derive(Debug)]
pub struct QuadTree<'a> {
    pub dimensions: [na::Point2<f32>; 2],
    pub subquads: Option<[Box<QuadTree<'a>>; 4]>,
    pub points: Vec<&'a PairedPoints>,
}

impl<'a> QuadTree<'a> {
    pub fn new(dim: [na::Point2<f32>; 2]) -> Self {
        QuadTree {
            dimensions: dim,
            subquads: None,
            points: Vec::with_capacity(2),
        }
    }

    // Takes a point a places into a quad tree; boxes are sub-divided into 4 equal sub quadrants when needed
    // Attempts to put a max of 2 points into each quad; recursivly sub-dividing until either there are <= 2 points
    // or the MIN_QUAD_DIM_SQUARED limit is reached, where there will then be >2 points in a box
    pub fn insert(&mut self, data: &'a PairedPoints) {
        const MIN_QUAD_DIM_SQUARED: f32 = 0.01;
        debug_assert!(
            data.coords.x >= self.dimensions[0].x
                && data.coords.x <= self.dimensions[1].x
                && data.coords.y >= self.dimensions[0].y
                && data.coords.y <= self.dimensions[1].y,
            "The given point value is outside of the tree dimensions"
        );
        if (self.subquads.is_none() && self.points.len() < 2)
            || na::distance_squared(&self.dimensions[0], &self.dimensions[1]) < MIN_QUAD_DIM_SQUARED
        {
            self.points.push(data);
        } else {
            if self.subquads.is_none() {
                // Create subquads
                self.subquads = Some([
                    Box::new(QuadTree::new([
                        na::point!(self.dimensions[0].x, self.dimensions[0].y),
                        na::point!(
                            (self.dimensions[0].x + self.dimensions[1].x) / 2f32,
                            (self.dimensions[0].y + self.dimensions[1].y) / 2f32
                        ),
                    ])),
                    Box::new(QuadTree::new([
                        na::point!(
                            self.dimensions[0].x,
                            (self.dimensions[0].y + self.dimensions[1].y) / 2f32
                        ),
                        na::point!(
                            (self.dimensions[0].x + self.dimensions[1].x) / 2f32,
                            self.dimensions[1].y
                        ),
                    ])),
                    Box::new(QuadTree::new([
                        na::point!(
                            (self.dimensions[0].x + self.dimensions[1].x) / 2f32,
                            (self.dimensions[0].y + self.dimensions[1].y) / 2f32
                        ),
                        na::point!(self.dimensions[1].x, self.dimensions[1].y),
                    ])),
                    Box::new(QuadTree::new([
                        na::point!(
                            (self.dimensions[0].x + self.dimensions[1].x) / 2f32,
                            self.dimensions[0].y
                        ),
                        na::point!(
                            self.dimensions[1].x,
                            (self.dimensions[0].y + self.dimensions[1].y) / 2f32
                        ),
                    ])),
                ]);
                // Move points into correct subquads
                while !self.points.is_empty() {
                    if let Some(mut s) = self.subquads.take() {
                        s[self.calc_quadrant(&self.points.last().unwrap().coords)]
                            .insert(self.points.last().unwrap().clone());
                        self.points.pop();
                        self.subquads = Some(s);
                    }
                }
            }
            assert!(self.subquads.is_some());
            // move new data point into correct subquad
            if let Some(mut s) = self.subquads.take() {
                s[self.calc_quadrant(&data.coords)].insert(data);
                self.subquads = Some(s);
            }
        }
    }

    // Return the correct subquadrant that the point should be placed into for the current box
    // Called on each level to put into the next subquad level
    pub fn calc_quadrant(&self, data: &na::Point2<f32>) -> usize {
        if data.x < (self.dimensions[0].x + self.dimensions[1].x) / 2.0 {
            if data.y < (self.dimensions[0].y + self.dimensions[1].y) / 2.0 {
                return 0;
            } else {
                return 1;
            }
        } else {
            if data.y < (self.dimensions[0].y + self.dimensions[1].y) / 2.0 {
                return 3;
            } else {
                return 2;
            }
        }
    }

    //Given a point get the point on another line that this point connects to
    pub fn get_neighboring_point(&mut self, data: &PairedPoints, tracked_points: &[bool]) -> usize {
        if let Some(mut s) = self.subquads.take() {
            let r = s[self.calc_quadrant(&data.coords)].get_neighboring_point(data, tracked_points);
            self.subquads = Some(s);
            r
        } else {
            if self.points.len() == 2 {
                for point in &self.points {
                    if data.id != point.id {
                        return point.id;
                    }
                }
                panic!("Could not find a valid point");
            } else {
                // The quad does not have the normal two points in a box, so we must guess what the correct connecting point is
                // Currently just find the closest point that we have not already been to.
                let mut dist: f32 = f32::MAX;
                let mut closest: &PairedPoints = self.points[0];
                for point in &self.points {
                    if data.id != point.id
                        && tracked_points[point.id] == false
                        && na::distance_squared(&data.coords, &point.coords) < dist
                    {
                        dist = na::distance_squared(&data.coords, &point.coords);
                        closest = point;
                    }
                }
                return closest.id;
            }
        }
    }
}

#[derive(Debug)]
pub struct PairedPoints {
    pub coords: na::Point2<f32>,
    pub id: usize,
    pub partner_index: Option<usize>,
}
impl PairedPoints {
    pub fn new(coord: na::Point2<f32>) -> Self {
        PairedPoints {
            coords: coord,
            id: 0,
            partner_index: None,
        }
    }
}
