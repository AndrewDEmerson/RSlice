use std::vec;

extern crate nalgebra as na;

#[derive(PartialEq, Debug)]
pub struct QuadTree {
    pub dimensions: [na::Point2<f32>; 2],
    pub subquads: Option<[Box<QuadTree>; 4]>,
    pub points: Vec<na::Point2<f32>>,
}

impl QuadTree {
    pub fn new(dim: [na::Point2<f32>; 2]) -> Self {
        QuadTree {
            dimensions: dim,
            subquads: None,
            points: Vec::with_capacity(2),
        }
    }
    pub fn insert(&mut self, data: na::Point2<f32>) {
        const MIN_QUAD_DIM_SQUARED: f32 = 0.01;
        debug_assert!(
            data.x >= self.dimensions[0].x
                && data.x <= self.dimensions[1].x
                && data.y >= self.dimensions[0].y
                && data.y <= self.dimensions[1].y,
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
                        s[self.calc_quadrant(self.points.last().unwrap())]
                            .insert(self.points.last().unwrap().clone());
                        self.points.pop();
                        self.subquads = Some(s);
                    }
                }
            }
            assert!(self.subquads.is_some());
            // move new data point into correct subquad
            if let Some(mut s) = self.subquads.take() {
                s[self.calc_quadrant(&data)].insert(data);
                self.subquads = Some(s);
            }
        }
    }
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
    pub fn get_points(&mut self) -> Vec<na::Point2<f32>> {
        if let Some(mut s) = self.subquads.take() {
            let mut ret: Vec<na::Point2<f32>> = Vec::new();
            ret.append(&mut s[0].get_points());
            ret.append(&mut s[1].get_points());
            ret.append(&mut s[2].get_points());
            ret.append(&mut s[3].get_points());
            self.subquads = Some(s);
            ret
        } else {
            let p = self.points.len();
            debug_assert!(
                self.points.len() == 0 || self.points.len() == 2,
                "Abnormal number of points in quad"
            );
            self.points.clone()
        }
    }
    pub fn get_pair(&mut self, data: na::Point2<f32>) -> Vec<na::Point2<f32>> {
        if let Some(mut s) = self.subquads.take() {
            let r = s[self.calc_quadrant(&data)].get_pair(data);
            self.subquads = Some(s);
            r
        } else {
            return self.points.clone();
        }
    }
}

#[cfg(test)]
#[test]
fn create() {
    let dim = [na::point!(0.0, 0.0), na::point!(10.0, 10.0)];
    let mut a = QuadTree::new(dim);
    for n in 0..100 {
        a.insert(na::point!(n as f32 / 10.0, 10.0 - (n as f32 / 10.0)));
        assert_eq!(a.get_points().len(), n + 1);
    }
}
