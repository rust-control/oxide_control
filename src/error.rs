pub enum Error {
    Mujoco(::rusty_mujoco::MjError),
    Mjs(String),
    NameNotFound(&'static str),
}

impl From<::rusty_mujoco::MjError> for Error {
    fn from(e: ::rusty_mujoco::MjError) -> Self {
        Error::Mujoco(e)
    }
}

impl std::fmt::Debug for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Mujoco(e) => write!(f, "Error::MuJoCo({e:?})"),
            Error::Mjs(msg) => write!(f, "Error::Mjs({msg})"),
            Error::NameNotFound(name) => write!(f, "Error::NameNotFound({name})"),
        }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Mujoco(e) => write!(f, "MuJoCo error: {e}"),
            Error::Mjs(msg) => write!(f, "MuJoCo error: {msg}"),
            Error::NameNotFound(name) => write!(f, "Given name not found: `{name}`"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Mujoco(e) => Some(e),
            Error::Mjs(_) => None,
            Error::NameNotFound(_) => None,
        }
    }
}
