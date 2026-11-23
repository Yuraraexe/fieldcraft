//! Triangle files read as imported surfaces ([`TaggedSurface`]): 3MF through FieldCraft's own
//! reader (which keeps SolidCraft's face tags, [`crate::tagged_mesh`]), STL, OBJ, PLY and glTF
//! through SolidCraft's mesh readers (`solidcraft_io::mesh_import_feature`), each mesh one surface
//! of one face. Which files these are comes from SolidCraft's format table
//! ([`crate::formats::Source::Triangles`]).
//!
//! An imported surface is triangles without a B-rep: shown, used for reference and as a fluid
//! obstacle, never volume-meshed (#1049). STL, OBJ and PLY carry no unit; they are read in
//! millimetres, as SolidCraft reads them.
//!
//! Files are untrusted: the file size and the vertex and triangle counts are capped (SolidCraft's
//! readers cap their own counts as they read, [`MAX_ELEMENTS`] caps the result), coordinates must
//! be finite and within [`MAX_COORD`], indices must exist, and the SolidCraft call runs under
//! `guard`, so a panic inside it is an error.

use solidcraft_kernel::{KernelError, guard};

use crate::tagged_mesh::{MAX_ELEMENTS, MAX_FILE_BYTES, TaggedSurface, read_3mf};
use crate::{Error, Result, SurfaceMesh};
use fieldcraft_model::MAX_COORD;

/// The surfaces of the triangle file `file` (its name selects the reader), in millimetres, each
/// carrying the file's name.
pub fn read_surfaces(file: &str, bytes: &[u8]) -> Result<Vec<TaggedSurface>> {
    let path = std::path::Path::new(file);
    let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| file.to_string());
    if crate::formats::source_of(file) != Some(crate::formats::Source::Triangles) {
        return Err(Error::Invalid(format!("`{name}` is not a triangle file FieldCraft reads (3MF, STL, OBJ, PLY, glTF)")));
    }
    if bytes.len() > MAX_FILE_BYTES {
        return Err(Error::Invalid(format!("`{name}`: file too large ({} MB)", bytes.len() >> 20)));
    }
    let is_3mf = path.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("3mf"));
    let mut surfaces = if is_3mf { read_3mf(bytes)? } else { solidcraft_meshes(file, bytes)? };
    for s in &mut surfaces {
        s.file.clone_from(&name);
    }
    Ok(surfaces)
}

/// An STL, OBJ, PLY or glTF file through SolidCraft's reader: one surface per mesh, one face each.
fn solidcraft_meshes(file: &str, bytes: &[u8]) -> Result<Vec<TaggedSurface>> {
    let feature = guard("read mesh file", || solidcraft_io::mesh_import_feature(bytes, file).map_err(|e| KernelError::Invalid(e.to_string())))?;
    let solidcraft_doc::FeatureKind::MeshImport { meshes, .. } = feature.kind else {
        return Err(Error::Failed(format!("`{file}` was not read as a mesh")));
    };
    let mut budget = 0usize;
    let mut out = Vec::with_capacity(meshes.len());
    for (k, m) in meshes.iter().enumerate() {
        let what = if m.name.trim().is_empty() { format!("Mesh{}", k + 1) } else { m.name.trim().to_string() };
        let (positions, []) = m.positions.as_chunks::<3>() else {
            return Err(Error::Invalid(format!("mesh `{what}`: an incomplete vertex")));
        };
        let (triangles, []) = m.triangles.as_chunks::<3>() else {
            return Err(Error::Invalid(format!("mesh `{what}`: an incomplete triangle")));
        };
        budget = budget.saturating_add(positions.len().max(triangles.len()));
        if budget > MAX_ELEMENTS {
            return Err(Error::Invalid(format!("more than {MAX_ELEMENTS} vertices or triangles")));
        }
        let positions = positions.to_vec();
        if positions.iter().flatten().any(|c| !(c.is_finite() && c.abs() <= MAX_COORD)) {
            return Err(Error::Invalid(format!("mesh `{what}`: a vertex is out of range")));
        }
        let triangles = triangles.to_vec();
        if triangles.is_empty() {
            return Err(Error::Invalid(format!("mesh `{what}` has no triangles")));
        }
        if let Some(v) = triangles.iter().flatten().find(|v| **v as usize >= positions.len()) {
            return Err(Error::Invalid(format!("mesh `{what}`: a triangle refers to vertex {v}, but there are {}", positions.len())));
        }
        let tri_face = vec![0; triangles.len()];
        out.push(TaggedSurface {
            name: what,
            file: String::new(),
            mesh: SurfaceMesh { positions, triangles, tri_face, ..SurfaceMesh::default() },
            faces: 1,
            original: None,
        });
    }
    if out.is_empty() {
        return Err(Error::Invalid(format!("`{file}` holds no mesh")));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::fmt::Write as _;

    use super::*;

    /// A unit tetrahedron: vertices, and its four faces wound outward.
    const TETRAHEDRON: [[f32; 3]; 4] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    const TET_TRIANGLES: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];

    fn tet_triangles() -> [[[f32; 3]; 3]; 4] {
        TET_TRIANGLES.map(|t| t.map(|v| TETRAHEDRON[v]))
    }

    fn ascii_stl() -> String {
        let mut s = String::from("solid tet\n");
        for t in tet_triangles() {
            s.push_str("facet normal 0 0 0\nouter loop\n");
            for [x, y, z] in t {
                writeln!(s, "vertex {x} {y} {z}").unwrap();
            }
            s.push_str("endloop\nendfacet\n");
        }
        s.push_str("endsolid tet\n");
        s
    }

    fn binary_stl(tris: &[[[f32; 3]; 3]]) -> Vec<u8> {
        let mut b = vec![0u8; 80];
        b.extend((tris.len() as u32).to_le_bytes());
        for t in tris {
            b.extend([0u8; 12]);
            for v in t {
                for c in v {
                    b.extend(c.to_le_bytes());
                }
            }
            b.extend([0u8; 2]);
        }
        b
    }

    const TET_OBJ: &str = "o Tet\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 1 4 3\n";

    const TET_PLY: &str = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n\
        element face 4\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n1 0 0\n0 1 0\n0 0 1\n3 0 2 1\n3 0 1 3\n3 1 2 3\n3 0 3 2\n";

    /// The enclosed volume: the sum of a·(b×c)/6 over the triangles.
    fn volume(m: &SurfaceMesh) -> f64 {
        m.triangles
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| m.positions[i as usize]);
                let cross = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
                a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]
            })
            .sum::<f64>()
            / 6.0
    }

    #[test]
    fn stl_obj_and_ply_read_as_one_face_surfaces() {
        let cases: [(&str, &str, Vec<u8>, &str); 4] = [
            ("dir/part.stl", "part.stl", ascii_stl().into_bytes(), "part"),
            ("dir/part.STL", "part.STL", binary_stl(&tet_triangles()), "part"),
            ("dir/part.obj", "part.obj", TET_OBJ.as_bytes().to_vec(), "Tet"),
            ("dir/part.ply", "part.ply", TET_PLY.as_bytes().to_vec(), "part"),
        ];
        for (path, file, bytes, name) in cases {
            let surfaces = read_surfaces(path, &bytes).unwrap();
            assert_eq!(surfaces.len(), 1, "{path}");
            let s = &surfaces[0];
            assert_eq!(s.name, name, "{path}");
            assert_eq!(s.file, file, "{path}");
            assert_eq!(s.faces, 1, "{path}");
            assert_eq!(s.mesh.triangles.len(), 4, "{path}");
            assert_eq!(s.mesh.tri_face, vec![0; 4], "{path}");
            assert!((volume(&s.mesh) - 1.0 / 6.0).abs() < 1e-6, "{path}: volume {}", volume(&s.mesh));
        }
    }

    #[test]
    fn garbage_and_truncated_files_are_errors() {
        let good_stl = ascii_stl().into_bytes();
        let one_triangle = binary_stl(&tet_triangles()[..1]);
        let mut huge_count = vec![0u8; 80];
        huge_count.extend(4_000_000_000u32.to_le_bytes());
        huge_count.extend([0u8; 50]);
        let random: Vec<u8> = (0..4096u32).map(|i| (i.wrapping_mul(2_654_435_761) >> 24) as u8).collect();
        let no_vertices = b"f 1 2 3\n".to_vec();
        let overflow_vertices = b"v 1e400 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n".to_vec();
        let cut_header = b"ply\nformat ascii 1.0\nelement vertex 3\n".to_vec();
        let huge_vertex_count =
            b"ply\nformat ascii 1.0\nelement vertex 4294967295\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n"
                .to_vec();
        let no_faces = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 0\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n".to_vec();
        let nan_stl =
            b"solid x\nfacet normal 0 0 0\nouter loop\nvertex nan 0 0\nvertex nan 1 0\nvertex nan 0 1\nendloop\nendfacet\nendsolid x\n".to_vec();
        // A binary STL has no signature, and SolidCraft reads one whose header count is wrong
        // (some writers leave 0) by the triangles that fit: random bytes named `.stl` are such a
        // file. Reading them must not fail badly: whatever is read is finite and in range.
        if let Ok(surfaces) = read_surfaces("x.stl", &random) {
            assert!(surfaces.iter().flat_map(|s| s.mesh.positions.iter().flatten()).all(|c| c.is_finite() && c.abs() <= MAX_COORD));
        }
        let cases: [(&str, Vec<u8>); 16] = [
            ("x.stl", Vec::new()),
            ("x.obj", Vec::new()),
            ("x.ply", Vec::new()),
            ("x.stl", vec![0u8; 84]),
            ("x.stl", one_triangle[..100].to_vec()),
            ("x.stl", huge_count),
            ("x.stl", b"solid x\nfacet\nendsolid".to_vec()),
            ("x.stl", nan_stl),
            ("x.obj", random.clone()),
            ("x.ply", random),
            ("x.obj", no_vertices),
            ("x.obj", overflow_vertices),
            ("x.ply", cut_header),
            ("x.ply", huge_vertex_count),
            ("x.ply", no_faces),
            ("x.step", good_stl.clone()),
        ];
        let mut accepted = Vec::new();
        for (name, bytes) in cases {
            if let Ok(surfaces) = read_surfaces(name, &bytes) {
                accepted.push(format!("{name} ({} bytes) read as {} surfaces", bytes.len(), surfaces.len()));
            }
        }
        if read_surfaces("x.json", &good_stl).is_ok() {
            accepted.push("x.json (the good ASCII STL) read".to_string());
        }
        assert!(accepted.is_empty(), "hostile input accepted: {accepted:#?}");
    }
}
