"""Utility functions and classes for robotic fabrication workflows.

This module provides general-purpose utility functions and classes including:
- Debug printing helpers
- Geometric calculations
- File system operations
- Logging utilities
- File finding by date patterns
"""
import logging
import os
import re
import datetime


def dprint(input, DPRINT=True):
    """Print debug information with context about the calling function.
    
    Provides enhanced debugging output that includes the name of the calling
    function and type information for collections. Only prints if DPRINT is True.
    
    Args:
        input: Any object to print. Can be a string, list, or other type.
        DPRINT (bool, optional): Enable/disable debug printing. Defaults to True.
        
    Examples:
        >>> dprint("Hello")  # Prints with calling function context
        >>> dprint([1, 2, 3], DPRINT=False)  # No output
    """
    if DPRINT is True:
        ''' print input in a more readable format and only if DPRINT is True
        '''
        import inspect

        print(
            "DPRINT called in "
            + inspect.stack()[1][3]
            + " ###############################################"
        )

        print(input)

        if isinstance(input, str):
            pass
            # print(input)
        else:
            try:
                print("length: " + str(len(input)))
                for things in input:
                    print("type: " + str(type(things)))
                    print(things)
            except Exception:
                pass

        print("END DPRINT###############################################")


def subtract_points(A, B, rounding=None):
    """Subtract coordinates of point B from point A element-wise.
    
    Performs vector subtraction A - B for 3D points, with optional rounding
    of the result.
    
    Args:
        A (tuple or list): First point as [x, y, z].
        B (tuple or list): Second point as [x, y, z].
        rounding (int, optional): Number of decimal places to round to.
            If None, no rounding is performed. Defaults to None.
            
    Returns:
        tuple: Resulting point (A[0]-B[0], A[1]-B[1], A[2]-B[2]).
        
    Examples:
        >>> subtract_points([1.5, 2.5, 3.5], [1.0, 2.0, 3.0])
        (0.5, 0.5, 0.5)
        >>> subtract_points([1.555, 2.555, 3.555], [1.0, 2.0, 3.0], rounding=2)
        (0.56, 0.56, 0.56)
    """
    if rounding:
        return (round(A[0]-B[0], rounding), round(A[1]-B[1], rounding), round(A[2]-B[2], rounding))
    return (A[0]-B[0], A[1]-B[1], A[2]-B[2])


def find_tool_values(xaxis, yaxis):
    """Generate tool frame values from X and Y axis vectors (Rhino/Grasshopper).
    
    Calculates the origin and axis vectors for a tool coordinate frame given
    the X and Y axis as Rhino line objects. Both vectors must start at the origin.
    
    Args:
        xaxis: Rhino Line object representing the tool's X axis.
        yaxis: Rhino Line object representing the tool's Y axis.
        
    Returns:
        None: Prints the origin and axis values to console.
        
    Notes:
        - Both xaxis and yaxis must start at the same point (origin)
        - Values are rounded to 4 decimal places
        - This function is designed for use in Grasshopper/Rhino environment
        
    Raises:
        ValueError: If vectors don't start at the same origin.
    """
    x_start = xaxis.PointAtStart
    x_end = xaxis.PointAtEnd
    y_start = yaxis.PointAtStart
    y_end = yaxis.PointAtEnd
    if not subtract_points(x_start, y_start, 0) == (0, 0, 0):
        print("both vectors must start at origin")
        return
    x = subtract_points(x_end, x_start, 4)
    y = subtract_points(y_end, y_start, 4)
    origin = subtract_points(x_start, [0, 0, 0], 4)
    print("origin: " + str(origin))
    print("x: " + str(x))
    print("y: " + str(y))

    return
    # print("origin: " + str(round(origin, 4)))


def get_data_path():
    """Get the absolute path to the project's data directory.
    
    Determines the data directory path based on execution context:
    - In Grasshopper: Uses ghdoc.Path to locate the data folder
    - In Python: Uses __file__ to navigate to ../../../rhino/../data
    
    Returns:
        str: Absolute path to the data directory.
        
    Notes:
        - Checks for 'ghdoc' in locals() to detect Grasshopper environment
        - Data directory is expected to be at project_root/data
    """
    import os
    if "ghdoc" in locals():
        print("getting data path from ghdoc")
        package_path = ghdoc.Path  # noqa: F821
        package_path = os.path.abspath(os.path.join(package_path, ".."))
    
    if os.path.basename(os.getcwd()) == "redundant_motion_planning":
        print("getting data path from cwd")
        return os.path.join(os.getcwd(), "data")
    else:
        print("getting data path from __file__")
        package_path = os.path.dirname(__file__)
        package_path = os.path.abspath(os.path.join(package_path, "../../../rhino"))
    data_path = os.path.abspath(os.path.join(package_path, "../data"))
    return data_path


class Logger:
    """Custom logging utility for robotic fabrication workflows.
    
    Provides structured logging to file with timestamp and log level information.
    Logs are written to auto_generated/compute_times.log in the data directory.
    
    Attributes:
        logging (bool): Flag to enable/disable logging.
        logger (logging.Logger): Python logger instance.
        
    Examples:
        >>> logger = Logger("/path/to/data")
        >>> logger.log("Task completed")
        >>> logger.warning("Check configuration")
        >>> logger.error("Process failed")
    """
    
    def __init__(self, data_path, logger_name="CustomLogger"):
        """Initialize the logger with file handler.
        
        Args:
            data_path (str): Path to data directory where logs will be stored.
            logger_name (str, optional): Name for the logger instance.
                Defaults to "CustomLogger".
        """
        self.logging = True
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.DEBUG)
        log_file = os.path.join(data_path, "auto_generated", "compute_times.log")

        # Prevent adding multiple handlers to the logger (if class is instantiated multiple times)
        if not self.logger.handlers:
            file_handler = logging.FileHandler(log_file)
            formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

    def log(self, message):
        """Log an info-level message.
        
        Args:
            message (str): Message to log.
        """
        if not self.logging:
            return
        self.logger.info(message)

    def debug(self, message):
        """Log a debug-level message.
        
        Args:
            message (str): Debug message to log.
        """
        self.logger.debug(message)

    def warning(self, message):
        """Log a warning-level message.
        
        Args:
            message (str): Warning message to log.
        """
        self.logger.warning(message)

    def error(self, message):
        """Log an error-level message.
        
        Args:
            message (str): Error message to log.
        """
        self.logger.error(message)

    def critical(self, message):
        """Log a critical-level message.
        
        Args:
            message (str): Critical error message to log.
        """
        self.logger.critical(message)


class FileFinder:
    """Utility for finding files by date pattern in filenames.
    
    Searches a directory for files matching a specific pattern and filetype,
    then returns either the youngest or oldest file based on timestamp in filename.
    
    Expected filename format: YYMMDD_HHMMSS_content.filetype
    Example: 250206_164749_print_task.script
    
    Attributes:
        directory (str): Directory path to search in.
        filetype (str): File extension to match (e.g., ".json", ".script").
        content (str): Content string to search for in filename.
        youngest (bool): If True, return youngest file; if False, oldest.
        
    Examples:
        >>> finder = FileFinder("/path/to/files", ".json", "solutions")
        >>> filepath = finder.get_file_by_date()
        >>> print(filepath)
        /path/to/files/250206_164749_solutions.json
    """
    
    def __init__(self, directory, filetype, content, youngest=True):
        """Initialize the FileFinder with search parameters.
        
        Args:
            directory (str): Path to directory to search.
            filetype (str): File extension to match (e.g., ".json").
            content (str): Content string that must appear in filename.
            youngest (bool, optional): Return youngest file if True, oldest if False.
                Defaults to True.
        """
        self.directory = directory
        self.filetype = filetype
        self.content = content
        self.youngest = youngest

    def get_file_by_date(self):
        """Find and return file path based on date in filename.
        
        Returns:
            str: Full path to the youngest or oldest matching file.
            
        Notes:
            - Delegates to get_youngest_file() or get_oldest_file()
            - Returns None if no matching files found
        """
        if self.youngest:
            return self.get_youngest_file()
        else:
            return self.get_oldest_file()

    def get_oldest_file(self):
        """Find the oldest file matching the search pattern.
        
        Scans directory for files matching the date pattern and content,
        returning the one with the earliest timestamp.
        
        Returns:
            str or None: Full path to oldest matching file, or None if not found.
        """
        oldest_file = None
        oldest_date = None
        pattern = re.compile(rf'\d{{6}}_\d{{6}}_{self.content}\.{self.filetype.lstrip(".")}')
        for filename in os.listdir(self.directory):
            if pattern.match(filename):
                date_str = filename[:12]
                date = datetime.datetime.strptime(date_str, '%y%m%d_%H%M%S')
                if oldest_date is None or date < oldest_date:
                    oldest_date = date
                    oldest_file = filename
        return os.path.join(self.directory, oldest_file) if oldest_file else None

    def get_youngest_file(self):
        """Find the most recent file matching the search pattern.
        
        Scans directory for files matching the date pattern and content,
        returning the one with the latest timestamp. Includes error handling
        for inaccessible directories.
        
        Returns:
            str or None: Full path to youngest matching file, or None if not found
                or directory is inaccessible.
        """
        youngest_file = None
        youngest_date = None
        pattern = re.compile(rf'\d{{6}}_\d{{6}}_{self.content}\.{self.filetype.lstrip(".")}')
        try:
            for filename in os.listdir(self.directory):
                if pattern.match(filename):
                    date_str = filename[:12]
                    date = datetime.datetime.strptime(date_str, '%y%m%d_%H%M%S')
                    if youngest_date is None or date > youngest_date:
                        youngest_date = date
                        youngest_file = filename
        except Exception as e:
            print(f"Error accessing directory {self.directory}: {e}")
            return None
        return os.path.join(self.directory, youngest_file) if youngest_file else None
