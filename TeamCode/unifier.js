const fs = require("fs");
const path = require("path");

const input = path.resolve(process.argv[2]);
const output = path.resolve(process.argv[3]);


const files = fs.readdirSync(input).filter(d => d.endsWith(".java"));
let code = "";
let imports = "";


for(const filename of files) {
	const fileLocation = path.join(input, filename)
	const file = fs.readFileSync(fileLocation).toString();
	console.log("Assimilating " + fileLocation);

	const lines = file.split("\n").forEach(d => {
		if(d.startsWith("import ")) {
			imports += d + "\n";
		} else if(d.startsWith("package ")) {
			return;
		} else {
			code += d + "\n";
		}
	});
}

fs.writeFileSync(output, imports + "\n\n" + code);
console.log("Unified "+files.length+" files to "+output);