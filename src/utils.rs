use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};

use crate::graph::Graph;
use crate::node::Node;
use crate::error::{RePathError, Result};
use crate::metrics::StatsSnapshot;

pub fn parse_obj(filename: &str) -> Result<Graph> {
    let file = File::open(filename)
        .map_err(|e| RePathError::NavmeshLoadError(format!("{}: {}", filename, e)))?;
    let reader = BufReader::new(file);

    let mut graph = Graph::new();
    let mut vertices: Vec<(f32, f32, f32)> = Vec::new();
    let mut vertex_id = 0;

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "v" => {
                let x: f32 = parts.get(1)
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid vertex X coordinate".to_string()))?;
                let y: f32 = parts.get(2)
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid vertex Y coordinate".to_string()))?;
                let z: f32 = parts.get(3)
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid vertex Z coordinate".to_string()))?;
                vertices.push((x, y, z));
                graph.add_node(Node::new(vertex_id, x, y, z));
                vertex_id += 1;
            }
            "f" => {
                let v1 = parts.get(1)
                    .and_then(|s| s.parse::<usize>().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid face vertex 1".to_string()))?
                    .checked_sub(1)
                    .ok_or_else(|| RePathError::ObjParseError("Face vertex index must be >= 1".to_string()))?;
                let v2 = parts.get(2)
                    .and_then(|s| s.parse::<usize>().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid face vertex 2".to_string()))?
                    .checked_sub(1)
                    .ok_or_else(|| RePathError::ObjParseError("Face vertex index must be >= 1".to_string()))?;
                let v3 = parts.get(3)
                    .and_then(|s| s.parse::<usize>().ok())
                    .ok_or_else(|| RePathError::ObjParseError("Invalid face vertex 3".to_string()))?
                    .checked_sub(1)
                    .ok_or_else(|| RePathError::ObjParseError("Face vertex index must be >= 1".to_string()))?;

                if v1 >= vertices.len() || v2 >= vertices.len() || v3 >= vertices.len() {
                    return Err(RePathError::ObjParseError("Face references invalid vertex".to_string()));
                }

                graph.add_edge(v1, v2, distance(&vertices[v1], &vertices[v2]));
                graph.add_edge(v2, v3, distance(&vertices[v2], &vertices[v3]));
                graph.add_edge(v3, v1, distance(&vertices[v3], &vertices[v1]));
            }
            _ => {}
        }
    }

    if graph.nodes.is_empty() {
        return Err(RePathError::NoNodesError);
    }

    Ok(graph)
}

pub fn distance(p1: &(f32, f32, f32), p2: &(f32, f32, f32)) -> f32 {
    let dx = p1.0 - p2.0;
    let dy = p1.1 - p2.1;
    let dz = p1.2 - p2.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn nodes_within_radius(graph: &Graph, node: &Node, radius: f32) -> Vec<usize> {
    graph
        .nodes
        .iter()
        .enumerate()
        .filter_map(|(id, n)| {
            let dist = distance(&(node.x, node.y, node.z), &(n.x, n.y, n.z));
            if dist <= radius {
                Some(id)
            } else {
                None
            }
        })
        .collect()
}

/// Save pathfinding metrics to a CSV file
///
/// If the file doesn't exist, it will be created with headers.
/// If the file exists, data will be appended to it.
///
/// This is the primary way to export metrics for analysis.
pub fn save_metrics_to_csv(filename: &str, stats: &StatsSnapshot) -> Result<()> {
    let file_exists = std::path::Path::new(filename).exists();

    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(filename)?;

    // Write header if this is a new file
    if !file_exists {
        writeln!(
            file,
            "total_requests,cache_hits,cache_misses,cache_hit_rate,early_exits_same_node,\
            early_exits_adjacent,nodes_explored,avg_nodes_explored,total_computation_time_us,\
            avg_computation_time_us,successful_paths,failed_paths,success_rate"
        )?;
    }

    // Write data row
    writeln!(
        file,
        "{},{},{},{:.2},{},{},{},{:.2},{},{:.2},{},{},{:.2}",
        stats.total_requests,
        stats.cache_hits,
        stats.cache_misses,
        stats.cache_hit_rate(),
        stats.early_exits_same_node,
        stats.early_exits_adjacent,
        stats.nodes_explored,
        stats.avg_nodes_explored(),
        stats.total_computation_time_us,
        stats.avg_computation_time_us(),
        stats.successful_paths,
        stats.failed_paths,
        stats.success_rate()
    )?;

    Ok(())
}
