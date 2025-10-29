use thiserror::Error;

/// Errors that can occur when using RePath
#[derive(Error, Debug)]
pub enum RePathError {
    /// Error when loading or parsing the navmesh file
    #[error("Failed to load navmesh file: {0}")]
    NavmeshLoadError(String),

    /// Error when parsing OBJ file format
    #[error("Failed to parse OBJ file: {0}")]
    ObjParseError(String),

    /// Error when no valid nodes exist in the graph
    #[error("No valid nodes found in navmesh")]
    NoNodesError,

    /// Error when a nearest node cannot be found
    #[error("Could not find nearest node to coordinates ({0}, {1}, {2})")]
    NearestNodeNotFound(f32, f32, f32),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// CSV error
    #[error("CSV error: {0}")]
    CsvError(#[from] csv::Error),
}

pub type Result<T> = std::result::Result<T, RePathError>;
