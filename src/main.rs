// VERSION FINALE ET COMPLÈTE
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use bevy::{
    core_pipeline::experimental::taa::TemporalAntiAliasBundle,
    math::DVec3,
    pbr::ScreenSpaceAmbientOcclusionBundle,
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
    tasks::{AsyncComputeTaskPool, Task, block_on},
};
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use gdal::vector::Geometry;
use std::path::{Path, PathBuf};

// --- Structs et Enums ---
#[derive(States, Debug, Clone, PartialEq, Eq, Hash, Default)]
enum AppState {
    #[default]
    SplashScreen,
    Main,
}
#[derive(Resource)]
struct SplashScreenTimer(Timer);
#[derive(Component)]
struct SplashScreenEntity;
#[derive(Component)]
struct AnimateSplashScreenText;

#[derive(Component)]
struct ParcelleLayer {
    pub class_id: u8,
    pub name: String,
    pub visible: bool,
}

#[derive(Debug, Clone)]
pub struct PointCloudData {
    pub class_id: u8,
    pub name: String,
    pub points: Vec<[f32; 3]>,
}
#[derive(Debug, Clone)]
pub struct MeshData {
    pub class_id: u8,
    pub name: String,
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

#[derive(Debug, Clone)]
enum ProcessedLayer {
    PointCloud(PointCloudData),
    Mesh(MeshData),
}

#[derive(Resource, Default, Clone)]
struct LoadedPoints(std::collections::HashMap<u8, Vec<DVec3>>);

#[derive(PartialEq, Clone, Copy, Debug)]
enum RenderMode {
    Points,
    Mesh,
}

#[derive(Resource)]
struct AppData {
    shapefile_path: Option<PathBuf>,
    lidar_paths: Vec<PathBuf>,
    is_loading: bool,
    render_mode: RenderMode,
    point_budget: f32,
    max_edge_length: f32,
    trigger_reprocess: bool,
    error_message: Option<String>,
}

impl Default for AppData {
    fn default() -> Self {
        Self {
            shapefile_path: Some(PathBuf::from("parcelles.shp")),
            lidar_paths: vec![
                PathBuf::from("chemin/vers/dalle1.laz"),
                PathBuf::from("chemin/vers/dalle2.laz"),
            ],
            is_loading: false,
            render_mode: RenderMode::Points,
            point_budget: 500_000.0,
            max_edge_length: 5.0,
            trigger_reprocess: false,
            error_message: None,
        }
    }
}

#[derive(Component)]
struct ComputeTask(Task<Option<Vec<ProcessedLayer>>>);
#[derive(Component)]
struct LoadPointsTask(Task<Result<LoadedPoints, String>>);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .init_state::<AppState>()
        .init_resource::<AppData>()
        .init_resource::<LoadedPoints>()
        .add_systems(OnEnter(AppState::SplashScreen), setup_splash)
        .add_systems(
            Update,
            (update_splash, animate_splash_text)
                .chain()
                .run_if(in_state(AppState::SplashScreen)),
        )
        .add_systems(OnExit(AppState::SplashScreen), cleanup_splash)
        .add_systems(OnEnter(AppState::Main), setup_main_scene)
        .add_systems(
            Update,
            (
                ui_system,
                handle_load_completion,
                handle_reprocess_completion,
                update_layer_visibility,
            )
                .chain()
                .run_if(in_state(AppState::Main)),
        )
        .run();
}

fn setup_splash(mut commands: Commands) {
    commands.spawn((Camera2dBundle::default(), SplashScreenEntity));
    commands
        .spawn((
            NodeBundle {
                style: Style {
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                background_color: Color::rgb(0.1, 0.1, 0.12).into(),
                ..default()
            },
            SplashScreenEntity,
        ))
        .with_children(|parent| {
            parent.spawn((
                TextBundle {
                    text: Text::from_section(
                        "VentaMaps",
                        TextStyle {
                            font_size: 100.0,
                            color: Color::WHITE.with_a(0.0),
                            ..default()
                        },
                    ),
                    transform: Transform::from_scale(Vec3::splat(0.8)),
                    ..default()
                },
                AnimateSplashScreenText,
            ));
        });
    commands.insert_resource(SplashScreenTimer(Timer::from_seconds(3.5, TimerMode::Once)));
}

fn animate_splash_text(
    timer: Res<SplashScreenTimer>,
    mut query: Query<(&mut Text, &mut Transform), With<AnimateSplashScreenText>>,
) {
    for (mut text, mut transform) in query.iter_mut() {
        let percent = (timer.0.elapsed_secs() / timer.0.duration().as_secs_f32()).min(1.0);
        let ease_percent = percent.powf(2.0);
        if let Some(section) = text.sections.get_mut(0) {
            section.style.color.set_a(ease_percent);
        }
        transform.scale = Vec3::splat(0.8 + 0.2 * ease_percent);
    }
}

fn update_splash(
    mut timer: ResMut<SplashScreenTimer>,
    time: Res<Time>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    if timer.0.tick(time.delta()).just_finished() {
        next_state.set(AppState::Main);
    }
}

fn cleanup_splash(mut commands: Commands, query: Query<Entity, With<SplashScreenEntity>>) {
    for entity in &query {
        commands.entity(entity).despawn_recursive();
    }
}

fn setup_main_scene(mut commands: Commands) {
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    let mut camera = commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-20.0, 20.5, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        PanOrbitCamera {
            button_pan: MouseButton::Left,
            modifier_pan: Some(KeyCode::ShiftLeft),
            ..default()
        },
    ));
    camera
        .insert(ScreenSpaceAmbientOcclusionBundle::default())
        .insert(TemporalAntiAliasBundle::default());
}

fn ui_system(
    mut commands: Commands,
    mut contexts: EguiContexts,
    mut app_data: ResMut<AppData>,
    loaded_points: Res<LoadedPoints>,
    mut query_layers: Query<(&mut ParcelleLayer, &Handle<StandardMaterial>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    egui::SidePanel::left("controls_panel").show(contexts.ctx_mut(), |ui| {
        ui.heading("Contrôles");
        ui.separator();
        if let Some(error) = &app_data.error_message {
            ui.colored_label(egui::Color32::RED, format!("ERREUR : {}", error));
            if ui.button("OK").clicked() {
                app_data.error_message = None;
            }
            ui.separator();
        }
        if ui.button("📂 Charger Parcelles (.shp)...").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Shapefile", &["shp"])
                .pick_file()
            {
                app_data.shapefile_path = Some(path);
            }
        }
        if let Some(path) = &app_data.shapefile_path {
            ui.monospace(path.display().to_string());
        }
        if ui.button("📂 Charger LiDAR (.laz)...").clicked() {
            if let Some(paths) = rfd::FileDialog::new()
                .add_filter("LiDAR", &["laz", "las"])
                .pick_files()
            {
                app_data.lidar_paths = paths;
            }
        }
        for path in &app_data.lidar_paths {
            ui.monospace(path.display().to_string());
        }
        ui.separator();
        let is_ready = app_data.shapefile_path.is_some() && !app_data.lidar_paths.is_empty();
        if ui
            .add_enabled(
                is_ready && !app_data.is_loading,
                egui::Button::new("📂 Charger les Données en Mémoire"),
            )
            .clicked()
        {
            app_data.is_loading = true;
            app_data.error_message = None;
            let thread_pool = AsyncComputeTaskPool::get();
            let shp_path = app_data.shapefile_path.clone().unwrap();
            let laz_paths = app_data.lidar_paths.clone();
            let task =
                thread_pool.spawn(async move { data_processing::load_points(shp_path, laz_paths) });
            commands.spawn(LoadPointsTask(task));
        }
        if app_data.is_loading {
            ui.spinner();
            ui.label("Chargement / Traitement...");
        }
        ui.separator();
        ui.heading("Rendu");
        if !loaded_points.0.is_empty() {
            let mut mode_changed = false;
            ui.horizontal(|ui| {
                mode_changed |= ui
                    .radio_value(
                        &mut app_data.render_mode,
                        RenderMode::Points,
                        "Nuage de Points",
                    )
                    .changed();
                mode_changed |= ui
                    .radio_value(
                        &mut app_data.render_mode,
                        RenderMode::Mesh,
                        "Surface (Triangulée)",
                    )
                    .changed();
            });
            if app_data.render_mode == RenderMode::Mesh {
                ui.label("Budget de points:");
                app_data.trigger_reprocess |= ui
                    .add(
                        egui::Slider::new(&mut app_data.point_budget, 1000.0..=2_000_000.0)
                            .logarithmic(true)
                            .text("points"),
                    )
                    .changed();
                ui.label("Longueur max des arêtes (m):");
                app_data.trigger_reprocess |= ui
                    .add(
                        egui::Slider::new(&mut app_data.max_edge_length, 1.0..=20.0).text("mètres"),
                    )
                    .changed();
            }
            if mode_changed {
                app_data.trigger_reprocess = true;
            }
            if ui
                .add_enabled(
                    !app_data.is_loading,
                    egui::Button::new("🔄 Appliquer les changements"),
                )
                .clicked()
            {
                app_data.trigger_reprocess = true;
            }
            if app_data.trigger_reprocess && !app_data.is_loading {
                app_data.is_loading = true;
                app_data.trigger_reprocess = false;
                let thread_pool = AsyncComputeTaskPool::get();
                let points_clone = loaded_points.clone();
                let params = data_processing::ReprocessParams {
                    render_mode: app_data.render_mode,
                    point_budget: app_data.point_budget as usize,
                    max_edge_length: app_data.max_edge_length as f64,
                };
                let task = thread_pool
                    .spawn(async move { data_processing::reprocess_points(points_clone, params) });
                commands.spawn(ComputeTask(task));
            }
        }
        ui.separator();
        ui.heading("Couches");
        for (mut layer, material_handle) in query_layers.iter_mut() {
            ui.horizontal(|ui| {
                ui.checkbox(&mut layer.visible, "");
                if let Some(material) = materials.get_mut(material_handle) {
                    let mut color_arr = material.base_color.as_rgba_f32();
                    if ui
                        .color_edit_button_rgba_unmultiplied(&mut color_arr)
                        .changed()
                    {
                        material.base_color =
                            Color::rgba(color_arr[0], color_arr[1], color_arr[2], color_arr[3]);
                    }
                }
                ui.label(format!("{} (Classe {})", layer.name, layer.class_id));
            });
        }
    });
}

fn update_layer_visibility(mut query: Query<(&ParcelleLayer, &mut Visibility)>) {
    for (layer, mut visibility) in query.iter_mut() {
        if layer.visible {
            *visibility = Visibility::Visible;
        } else {
            *visibility = Visibility::Hidden;
        }
    }
}

fn handle_load_completion(
    mut commands: Commands,
    mut loaded_points: ResMut<LoadedPoints>,
    mut app_data: ResMut<AppData>,
    mut tasks: Query<(Entity, &mut LoadPointsTask)>,
) {
    for (entity, mut task) in &mut tasks {
        if let Some(result) = block_on(futures_lite::future::poll_once(&mut task.0)) {
            commands.entity(entity).despawn();
            app_data.is_loading = false;
            match result {
                Ok(points) => {
                    *loaded_points = points;
                    app_data.trigger_reprocess = true;
                }
                Err(e) => {
                    app_data.error_message = Some(e);
                    *loaded_points = LoadedPoints::default();
                }
            }
        }
    }
}

fn handle_reprocess_completion(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut app_data: ResMut<AppData>,
    mut tasks: Query<(Entity, &mut ComputeTask)>,
    old_parcelles: Query<Entity, With<ParcelleLayer>>,
) {
    for (entity, mut task) in &mut tasks {
        if let Some(result) = block_on(futures_lite::future::poll_once(&mut task.0)) {
            commands.entity(entity).despawn();
            app_data.is_loading = false;
            for old_entity in &old_parcelles {
                commands.entity(old_entity).despawn();
            }
            if let Some(layers_data) = result {
                for layer in layers_data {
                    let (mesh_data, material_data, class_id, name) = match layer {
                        ProcessedLayer::PointCloud(data) => {
                            let mut mesh = Mesh::new(
                                PrimitiveTopology::PointList,
                                RenderAssetUsages::default(),
                            );
                            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, data.points);
                            (
                                mesh,
                                StandardMaterial {
                                    unlit: true,
                                    ..default()
                                },
                                data.class_id,
                                data.name,
                            )
                        }
                        ProcessedLayer::Mesh(data) => {
                            let mut mesh = Mesh::new(
                                PrimitiveTopology::TriangleList,
                                RenderAssetUsages::default(),
                            );
                            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, data.vertices);
                            mesh.insert_indices(Indices::U32(data.indices));
                            mesh.duplicate_vertices();
                            mesh.compute_flat_normals();
                            (
                                mesh,
                                StandardMaterial {
                                    perceptual_roughness: 1.0,
                                    ..default()
                                },
                                data.class_id,
                                data.name,
                            )
                        }
                    };

                    let mut material = material_data;
                    material.base_color = data_processing::get_color_for_class(class_id);
                    commands.spawn((
                        PbrBundle {
                            mesh: meshes.add(mesh_data),
                            material: materials.add(material),
                            ..default()
                        },
                        ParcelleLayer {
                            class_id,
                            name,
                            visible: true,
                        },
                    ));
                }
            }
        }
    }
}

mod data_processing {
    use super::{
        Color, Geometry, LoadedPoints, MeshData, Path, PathBuf, PointCloudData, ProcessedLayer,
        RenderMode,
    };
    use bevy::math::DVec3;
    use gdal::Dataset;
    use gdal::vector::{FieldValue, LayerAccess};
    use geo::{Contains, Point, Polygon};
    use las::{Read, Reader, point::Classification};
    use spade::{DelaunayTriangulation, Point2, Triangulation};
    use std::collections::HashMap;

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
                        let vertices: Vec<[f32; 3]> = vertices_dvec
                            .iter()
                            .map(|p| (*p - center).as_vec3().to_array())
                            .collect();
                        layers_data.push(ProcessedLayer::Mesh(MeshData {
                            class_id,
                            name,
                            vertices,
                            indices,
                        }));
                    }
                }
            }
        }
        Some(layers_data)
    }

    pub fn load_points(shp_path: PathBuf, laz_paths: Vec<PathBuf>) -> Result<LoadedPoints, String> {
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
            let mut reader = Reader::from_path(chemin_lidar)
                .map_err(|e| format!("Impossible d'ouvrir {:?}: {}", chemin_lidar, e))?;
            for point_result in reader.points() {
                let point = point_result.map_err(|e| e.to_string())?;
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
            Err(
                "Aucun point LiDAR n'a été trouvé à l'intérieur de la géométrie de la parcelle."
                    .to_string(),
            )
        } else {
            Ok(LoadedPoints(points_par_classe))
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
    fn charger_geometrie_parcelle(
        chemin: &Path,
        champ_id: &str,
        id_cible: &str,
    ) -> Result<Geometry, String> {
        let dataset = Dataset::open(chemin).map_err(|e| {
            format!(
                "Fichier SHP non trouvé ou illisible : {:?}. Erreur: {}",
                chemin, e
            )
        })?;
        let mut layer = dataset.layer(0).unwrap();
        for feature in layer.features() {
            if let Ok(Some(FieldValue::StringValue(id_lue))) = feature.field(champ_id) {
                if id_lue == id_cible {
                    if let Some(geom) = feature.geometry() {
                        return Ok(geom.clone());
                    }
                }
            }
        }
        Err(format!(
            "Parcelle avec l'ID '{}' non trouvée dans le fichier {:?}",
            id_cible, chemin
        ))
    }
}
