pub enum Error {
    Mujoco(::rusty_mujoco::MjError),
    Mjs(String),
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
        }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Mujoco(e) => write!(f, "MuJoCo error: {e}"),
            Error::Mjs(msg) => write!(f, "MuJoCo error: {msg}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Mujoco(e) => Some(e),
            Error::Mjs(_) => None,
        }
    }
}
