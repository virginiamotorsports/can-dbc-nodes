import cantools, os
import numpy as np

def dbc_to_c(dbc_file):
    # Load the DBC file
    db = cantools.database.load_file(dbc_file)

    # Generate C structs
    c_code = ""
    for message in db.messages:
        c_code += f"struct {message.name} {{\n"
        print(message.name)
        for signal in message.signals:
            print(type(signal))
            c_code += f"    {signal.type} {signal.name};\n"
        c_code += "};\n\n"
    return c_code



if __name__ == "__main__":
    dbc_file = "../msg_publisher/launch/master_dbc.dbc"
    c_code = dbc_to_c(dbc_file)
    print(c_code)
    # Save the generated C code to a file
    output_file = "/path/to/output/file.c"
    with open(output_file, "w") as f:
        f.write(c_code)
    print(f"C code saved to {output_file}")