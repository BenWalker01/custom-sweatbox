/// FSD message type
#[derive(Debug, Clone)]
pub struct FsdMessage {
    pub raw: String,
    pub parts: Vec<String>,
}

impl FsdMessage {
    pub fn parse(raw: String) -> Self {
        let parts: Vec<String> = raw.split(':').map(|s| s.to_string()).collect();
        Self { raw, parts }
    }

    pub fn get(&self, index: usize) -> Option<&str> {
        self.parts.get(index).map(|s| s.as_str())
    }

    pub fn encode(parts: &[&str]) -> String {
        format!("{}\r\n", parts.join(":"))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientType {
    Controller,
    Pilot,
}
