// src/data_processing.rs

use crate::{Color, LoadedPoints, MeshData, PointCloudData, ProcessedLayer, RenderMode};
use bevy::math::DVec3;
use gdal::{
    Dataset,
    vector::{FieldValue, Geometry, LayerAccess},
};
use geo::{Contains, Point, Polygon};
use las::{Read, Reader, point::Classification};
use spade::{DelaunayTriangulation, Point2, Triangulation};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub struct ReprocessParams {
    pub render_mode: RenderMode,
    pub point_budget: usize,
    pub max_edge_length: f64,
}

pub fn reprocess_points(
    loaded_points: LoadedPoints,
    params: ReprocessParams,
) -> Option<Vec<ProcessedLayer>> {
    let center = calculate_center(&loaded_points.0);
    let mut layers_data = Vec::new();

    for (class_id, points) in loaded_points.0 {
        let name = get_name_for_class(class_id);
        let budget = if params.point_budget > 0 && params.point_budget < points.len() {
            params.point_budget
        } else {
            points.len()
        };
        let step = (points.len() as f32 / budget as f32).ceil() as usize;
        let points_budgeted: Vec<DVec3> = points.iter().step_by(step).cloned().collect();

        match params.render_mode {
            RenderMode::Points => {
                let vertices: Vec<[f32; 3]> = points_budgeted
                    .iter()
                    .map(|p| (*p - center).as_vec3().to_array())
                    .collect();
                layers_data.push(ProcessedLayer::PointCloud(PointCloudData {
                    class_id,
                    name,
                    points: vertices,
                }));
            }
            RenderMode::Mesh => {
                if (class_id == u8::from(Classification::Ground)
                    || class_id == u8::from(Classification::Building))
                    && points_budgeted.len() >= 3
                {
                    let (vertices_dvec, indices) =
                        triangulate(&points_budgeted, params.max_edge_length);
                    let vertices_f32: Vec<[f32; 3]> = vertices_dvec
                        .iter()
                        .map(|p| (*p - center).as_vec3().to_array())
                        .collect();
                    layers_data.push(ProcessedLayer::Mesh(MeshData {
                        class_id,
                        name,
                        vertices: vertices_f32,
                        indices,
                    }));
                }
            }
        }
    }
    Some(layers_data)
}

pub fn load_points(shp_path: PathBuf, laz_paths: Vec<PathBuf>) -> Option<LoadedPoints> {
    let geom = charger_geometrie_parcelle(&shp_path, "id", "84073000AW0443")?;
    let ring_gdal = geom.get_geometry(0);
    let points_2d_geo: Vec<(f64, f64)> = ring_gdal
        .get_point_vec()
        .into_iter()
        .map(|(x, y, _z)| (x, y))
        .collect();
    let polygon_geo = Polygon::new(points_2d_geo.into(), vec![]);
    let mut points_par_classe: HashMap<u8, Vec<DVec3>> = HashMap::new();

    for chemin_lidar in &laz_paths {
        let mut reader = Reader::from_path(chemin_lidar).ok()?;
        for point_result in reader.points() {
            let point = point_result.ok()?;
            let point_geo = Point::new(point.x, point.y);
            if polygon_geo.contains(&point_geo) {
                let classe = u8::from(point.classification);
                points_par_classe
                    .entry(classe)
                    .or_default()
                    .push(DVec3::new(point.x, point.y, point.z));
            }
        }
    }
    if points_par_classe.is_empty() {
        None
    } else {
        Some(LoadedPoints(points_par_classe))
    }
}

fn triangulate(points: &[DVec3], max_edge_length: f64) -> (Vec<DVec3>, Vec<u32>) {
    let point_to_index_map: HashMap<(u64, u64), usize> = points
        .iter()
        .enumerate()
        .map(|(i, p)| ((p.x.to_bits(), p.y.to_bits()), i))
        .collect();
    let mut triangulation = DelaunayTriangulation::<Point2<f64>>::new();
    for p in points {
        let _ = triangulation.insert(Point2::new(p.x, p.y));
    }
    let indices: Vec<u32> = triangulation
        .inner_faces()
        .filter_map(|face| {
            let v = face.vertices();
            let p0_val = v[0].position();
            let p1_val = v[1].position();
            let p2_val = v[2].position();
            let d01 = ((p0_val.x - p1_val.x).powi(2) + (p0_val.y - p1_val.y).powi(2)).sqrt();
            let d12 = ((p1_val.x - p2_val.x).powi(2) + (p1_val.y - p2_val.y).powi(2)).sqrt();
            let d20 = ((p2_val.x - p0_val.x).powi(2) + (p2_val.y - p0_val.y).powi(2)).sqrt();
            if d01 <= max_edge_length && d12 <= max_edge_length && d20 <= max_edge_length {
                let p0_idx = *point_to_index_map
                    .get(&(p0_val.x.to_bits(), p0_val.y.to_bits()))
                    .unwrap();
                let p1_idx = *point_to_index_map
                    .get(&(p1_val.x.to_bits(), p1_val.y.to_bits()))
                    .unwrap();
                let p2_idx = *point_to_index_map
                    .get(&(p2_val.x.to_bits(), p2_val.y.to_bits()))
                    .unwrap();
                Some([p0_idx as u32, p1_idx as u32, p2_idx as u32])
            } else {
                None
            }
        })
        .flatten()
        .collect();
    (points.to_vec(), indices)
}

fn calculate_center(points_map: &HashMap<u8, Vec<DVec3>>) -> DVec3 {
    let all_points: Vec<DVec3> = points_map.values().flatten().cloned().collect();
    if all_points.is_empty() {
        return DVec3::ZERO;
    }
    all_points.iter().fold(DVec3::ZERO, |acc, p| acc + *p) / all_points.len() as f64
}

pub fn get_color_for_class(class_id: u8) -> Color {
    match class_id {
        2 => Color::rgb(0.4, 0.6, 0.2),
        6 => Color::rgb(0.7, 0.3, 0.2),
        3..=5 => Color::rgb(0.2, 0.5, 0.1),
        _ => Color::rgb(0.5, 0.5, 0.5),
    }
}

fn get_name_for_class(class_id: u8) -> String {
    match class_id {
        2 => "Sol".to_string(),
        6 => "Bâtiment".to_string(),
        3..=5 => "Végétation".to_string(),
        c => format!("Classe {}", c),
    }
}

fn charger_geometrie_parcelle(chemin: &Path, champ_id: &str, id_cible: &str) -> Option<Geometry> {
    let dataset = Dataset::open(chemin).ok()?;
    let mut layer = dataset.layer(0).ok()?;
    for feature in layer.features() {
        if let Some(FieldValue::StringValue(id_lue)) = feature.field(champ_id).unwrap_or(None) {
            if id_lue == id_cible {
                return feature.geometry().cloned();
            }
        }
    }
    None
}
