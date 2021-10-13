use byteorder::{ByteOrder, LittleEndian};
use std::fmt;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;

/// PointCloud struct
/// 
/// Usage:
/// 
/// ```
/// use pcd_reader::PointCloud;
/// let filename = "sample/sample_binary_compressed.pcd";
/// let pcd = PointCloud::from_filename(filename);
/// let x_data = pcd.get_data_f32("x");
/// let y_data = pcd.get_data_f32("y");
/// let z_data = pcd.get_data_f32("z");
/// let intensity_data = pcd.get_data_u8("intensity");
/// let ring_data = pcd.get_data_u8("ring");
/// assert_eq!(pcd.header.data_format, "binary_compressed");
/// assert_eq!(pcd.header.num_points, 159602);
/// assert_eq!(pcd.header.field_names, ["x", "y", "z", "intensity", "ring"]);
/// assert_eq!(pcd.header.size_list, [4, 4, 4, 1, 1]);
/// assert_eq!(pcd.header.type_list, ["F", "F", "F", "U", "U"]);
/// assert_eq!(pcd.decompressed_buffer.len(), 2234428);
/// assert_eq!(x_data.len(), 159602);
/// assert_eq!(y_data.len(), 159602);
/// assert_eq!(z_data.len(), 159602);
/// assert_eq!(intensity_data.len(), 159602);
/// assert_eq!(ring_data.len(), 159602);
/// ```
pub struct PointCloud {
    pub header: PointCloudHeader,
    pub decompressed_buffer: Vec<u8>,
}

impl fmt::Debug for PointCloud {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PointCloud")
            .field("header", &self.header)
            .field(
                "decompressed_buffer",
                &format!("[{} bytes buffer]", self.decompressed_buffer.len()),
            )
            .finish()
    }
}

#[derive(Debug)]
pub struct PointCloudHeader {
    pub data_format: String,
    pub num_points: usize,
    pub field_names: Vec<String>,
    pub size_list: Vec<usize>,
    pub type_list: Vec<String>,
}

impl PointCloud {
    pub fn from_filename(filename: &str) -> PointCloud {
        let f = File::open(filename).expect("error reading pcd file.");
        let mut reader = BufReader::new(&f);
        let data_format: String;
        let mut num_points: usize = 0;
        let mut field_names = Vec::<String>::new();
        let mut size_list = Vec::<usize>::new();
        let mut type_list = Vec::<String>::new();

        loop {
            let mut line = String::new();
            let _ = reader
                .read_line(&mut line)
                .expect("failed to read a line from pcd header.");
            line = line[..line.len() - 1].to_string();

            if line.starts_with("#") {
            } else if line.starts_with("VERSION") {
            } else if line.starts_with("FIELDS") {
                let words: Vec<String> = line.split(" ").map(|s| s.to_string()).collect();
                let words = &words[1..];
                field_names = words.to_vec();
            } else if line.starts_with("SIZE") {
                let words: Vec<String> = line.split(" ").map(|s| s.to_string()).collect();
                let words = &words[1..];
                size_list = words.iter().map(|s|s.parse::<usize>().expect("entry POINTS in header has wrong format: its second term is not integer.")).collect();
            } else if line.starts_with("TYPE") {
                let words: Vec<String> = line.split(" ").map(|s| s.to_string()).collect();
                let words = &words[1..];
                type_list = words.to_vec();
            } else if line.starts_with("COUNT") {
            } else if line.starts_with("WIDTH") {
            } else if line.starts_with("HEIGHT") {
            } else if line.starts_with("VIEWPOINT") {
            } else if line.starts_with("POINTS") {
                let words: Vec<&str> = line.split(" ").collect();
                if words.len() != 2 {
                    panic!("entry POINTS in header has wrong format: it consists of other than 2 words")
                }
                num_points = words[1].parse::<usize>().expect(
                    "entry POINTS in header has wrong format: its second term is not integer.",
                );
            } else if line.starts_with("DATA") {
                let words: Vec<&str> = line.split(" ").collect();
                if words.len() != 2 {
                    panic!(
                        "entry DATA in header has wrong format: it consists of other than 2 words"
                    )
                }
                if "binary_compressed" == words[1] {
                    data_format = words[1].to_string();
                } else {
                    panic!("currently only supporting binary_compressed format.");
                }
                break;
            } else {
                panic!("unknown header entry");
            }
        }

        let mut u32_size_buffer = vec![0u8; 4];
        let _ = reader.read_exact(&mut u32_size_buffer);
        let compressed_size = LittleEndian::read_u32(&u32_size_buffer) as usize;
        let _ = reader.read_exact(&mut u32_size_buffer);
        let uncompressed_size = LittleEndian::read_u32(&u32_size_buffer) as usize;

        let mut compressed_size_buffer = vec![0u8; compressed_size];
        let _ = reader.read_exact(&mut compressed_size_buffer);
        let decompressed_buffer = lzf::decompress(&compressed_size_buffer, uncompressed_size)
            .expect("error decompressing binary_comprressed pcd data.");
        PointCloud {
            decompressed_buffer,
            header: PointCloudHeader {
                data_format,
                num_points,
                field_names,
                size_list,
                type_list,
            },
        }
    }

    fn get_data_offset(&self, fieldname: &str, type_string: &str, item_size: usize) -> usize {
        let mut data_offset: usize = 0;
        for (i, fname) in self.header.field_names.iter().enumerate() {
            if fname == fieldname {
                if self.header.type_list[i] != type_string || self.header.size_list[i] != item_size
                {
                    panic!(
                        "required fieldname is not a type of {}{}",
                        type_string, item_size
                    )
                }
                break;
            }
            data_offset += self.header.size_list[i];
        }
        data_offset
    }

    fn read_data<T>(
        &self,
        fieldname: &str,
        type_string: &str,
        item_size: usize,
        read_buffer_fn: fn(&[u8], &mut [T]),
        data_buffer: &mut Vec<T>,
    ) {
        if !self.header.field_names.contains(&fieldname.to_string()) {
            panic!("pointcloud does not contain required fieldname");
        }
        let data_offset = self.get_data_offset(fieldname, type_string, item_size);
        read_buffer_fn(
            &self.decompressed_buffer[data_offset * self.header.num_points
                ..(data_offset + item_size) * self.header.num_points],
            data_buffer,
        );
    }

    pub fn get_data_f32(&self, fieldname: &str) -> Vec<f32> {
        let mut data_buffer = vec![0.0; self.header.num_points];
        self.read_data::<f32>(
            fieldname,
            "F",
            4,
            LittleEndian::read_f32_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_f64(&self, fieldname: &str) -> Vec<f64> {
        let mut data_buffer = vec![0.0; self.header.num_points];
        self.read_data::<f64>(
            fieldname,
            "F",
            8,
            LittleEndian::read_f64_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_u8(&self, fieldname: &str) -> Vec<u8> {
        let mut data_buffer = vec![0; self.header.num_points];
        fn copy_u8_into(source: &[u8], target: &mut [u8]) {
            target[..].clone_from_slice(source);
        }
        self.read_data::<u8>(fieldname, "U", 1, copy_u8_into, &mut data_buffer);
        data_buffer
    }

    pub fn get_data_u16(&self, fieldname: &str) -> Vec<u16> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<u16>(
            fieldname,
            "U",
            2,
            LittleEndian::read_u16_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_u32(&self, fieldname: &str) -> Vec<u32> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<u32>(
            fieldname,
            "U",
            4,
            LittleEndian::read_u32_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_u64(&self, fieldname: &str) -> Vec<u64> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<u64>(
            fieldname,
            "U",
            8,
            LittleEndian::read_u64_into,
            &mut data_buffer,
        );
        data_buffer
    }

    // pub fn get_data_i8(&self, fieldname: &str) -> Vec<i8> {
    //     let mut data_buffer = vec![0; self.header.num_points];
    //     self.read_data::<i8>(
    //         fieldname,
    //         "I",
    //         1,
    //         LittleEndian::read_i8_into,
    //         &mut data_buffer,
    //     );
    //     data_buffer
    // }

    pub fn get_data_i16(&self, fieldname: &str) -> Vec<i16> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<i16>(
            fieldname,
            "I",
            2,
            LittleEndian::read_i16_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_i32(&self, fieldname: &str) -> Vec<i32> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<i32>(
            fieldname,
            "I",
            4,
            LittleEndian::read_i32_into,
            &mut data_buffer,
        );
        data_buffer
    }

    pub fn get_data_i64(&self, fieldname: &str) -> Vec<i64> {
        let mut data_buffer = vec![0; self.header.num_points];
        self.read_data::<i64>(
            fieldname,
            "I",
            8,
            LittleEndian::read_i64_into,
            &mut data_buffer,
        );
        data_buffer
    }
}

