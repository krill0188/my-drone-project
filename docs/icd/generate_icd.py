#!/usr/bin/env python3
"""
ICD Documentation Generator
Generates ICD documentation from MAVLink/ROS 2 contracts
Usage: python3 generate_icd.py
"""

import json
import os
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any

CONTRACTS_DIR = Path(__file__).parent.parent / "contracts"
OUTPUT_DIR = Path(__file__).parent.parent / "docs" / "icd"

class ICDGenerator:
    def __init__(self):
        self.messages = {}
        self.services = {}
        self.enums = {}
        
    def load_contracts(self):
        """Load MAVLink/ROS 2 contract definitions"""
        # MAVLink messages from common.xml
        mavlink_dir = Path(__file__).parent.parent / "contracts" / "mavlink"
        if mavlink_dir.exists():
            self._parse_mavlink(mavlink_dir)
        
        # ROS 2 messages from .msg files
        ros2_dir = Path(__file__).parent.parent / "contracts" / "ros2"
        if ros2_dir.exists():
            self._parse_ros2(ros2_dir)
            
        # Swarm custom messages
        swarm_dir = Path(__file__).parent.parent / "contracts" / "swarm"
        if swarm_dir.exists():
            self._parse_swarm(swarm_dir)
    
    def _parse_mavlink(self, dir_path: Path):
        """Parse MAVLink common.xml"""
        # Simplified - would use pymavlink in reality
        pass
    
    def _parse_ros2(self, dir_path: Path):
        """Parse ROS 2 .msg files"""
        for msg_file in dir_path.rglob("*.msg"):
            with open(msg_file) as f:
                content = f.read()
                # Parse ROS 2 message format
                self._parse_ros2_msg(msg_file.stem, content)
    
    def _parse_swarm(self, dir_path: Path):
        """Parse swarm custom message definitions"""
        for json_file in dir_path.rglob("*.json"):
            with open(json_file) as f:
                data = json.load(f)
                self.messages[json_file.stem] = data
    
    def _parse_ros2_msg(self, name: str, content: str):
        """Parse ROS 2 message definition"""
        fields = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                fields.append({
                    'type': parts[0],
                    'name': parts[1],
                    'description': ' '.join(parts[2:]) if len(parts) > 2 else ''
                })
        self.messages[name] = {'fields': fields, 'source': 'ros2'}
    
    def generate_markdown(self):
        """Generate ICD markdown documentation"""
        output = []
        output.append("# Interface Control Document (ICD)")
        output.append(f"**Generated**: {datetime.now().isoformat()}")
        output.append(f"**Version**: 1.0.0")
        output.append("")
        output.append("---")
        output.append("")
        
        # MAVLink Messages
        output.append("## MAVLink Messages")
        output.append("")
        for name, msg in self.messages.items():
            if msg.get('source') == 'mavlink':
                output.append(f"### {name}")
                output.append(f"- **ID**: {msg.get('id', 'N/A')}")
                output.append(f"- **CRC**: {msg.get('crc', 'N/A')}")
                output.append(f"- **Fields**:")
                for field in msg.get('fields', []):
                    output.append(f"  - `{field['name']}`: {field['type']} - {field.get('description', '')}")
                output.append("")
        
        # ROS 2 Messages
        output.append("## ROS 2 Messages")
        output.append("")
        for name, msg in self.messages.items():
            if msg.get('source') == 'ros2':
                output.append(f"### {name}")
                output.append("| Field | Type | Description |")
                output.append("|-------|------|-------------|")
                for field in msg.get('fields', []):
                    output.append(f"| {field['name']} | {field['type']} | {field.get('description', '')} |")
                output.append("")
        
        # Swarm Custom Messages
        output.append("## Swarm Custom Messages")
        output.append("")
        for name, msg in self.messages.items():
            if msg.get('source') == 'swarm':
                output.append(f"### {name}")
                output.append("| Field | Type | Description |")
                output.append("|-------|------|-------------|")
                for field in msg.get('fields', []):
                    output.append(f"| {field['name']} | {field['type']} | {field.get('description', '')} |")
                output.append("")
        
        # Services
        output.append("## Services")
        output.append("")
        for name, svc in self.services.items():
            output.append(f"### {name}")
            output.append("**Request**")
            output.append("| Field | Type | Description |")
            output.append("|-------|------|-------------|")
            for field in svc.get('request', []):
                output.append(f"| {field['name']} | {field['type']} | {field.get('description', '')} |")
            output.append("")
            output.append("**Response**")
            output.append("| Field | Type | Description |")
            output.append("|-------|------|-------------|")
            for field in svc.get('response', []):
                output.append(f"| {field['name']} | {field['type']} | {field.get('description', '')} |")
            output.append("")
        
        return '\n'.join(output)
    
    def generate_html(self):
        """Generate HTML documentation"""
        md = self.generate_markdown()
        # Simple markdown to HTML conversion
        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>AI Swarm ICD Documentation</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }}
        h1 {{ color: #2c3e50; border-bottom: 2px solid #3498db; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        h3 {{ color: #2980b9; }}
        table {{ border-collapse: collapse; width: 100%; margin: 20px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 12px; text-align: left; }}
        th {{ background-color: #34495e; color: white; }}
        tr:nth-child(even) {{ background-color: #f2f2f2; }}
        code {{ background: #f4f4f4; padding: 2px 6px; border-radius: 3px; }}
        pre {{ background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 5px; overflow-x: auto; }}
        .toc {{ background: #ecf0f1; padding: 20px; border-radius: 5px; margin-bottom: 30px; }}
        .toc a {{ text-decoration: none; color: #2980b9; }}
        .toc a:hover {{ text-decoration: underline; }}
    </style>
</head>
<body>
{self._markdown_to_html(md)}
</body>
</html>"""
        return html
    
    def _markdown_to_html(self, md: str) -> str:
        """Simple markdown to HTML conversion"""
        html = md
        # Headers
        html = html.replace('\n# ', '\n<h1>').replace('\n## ', '\n<h2>').replace('\n### ', '\n<h3>')
        html = html.replace('\n#### ', '\n<h4>')
        # Close headers
        lines = html.split('\n')
        for i, line in enumerate(lines):
            if line.startswith('<h'):
                lines[i] = line + '</' + line[1:3] + '>'
        html = '\n'.join(lines)
        # Code blocks
        html = html.replace('```', '<pre><code>').replace('```', '</code></pre>')
        # Inline code
        import re
        html = re.sub(r'`([^`]+)`', r'<code>\1</code>', html)
        # Bold
        html = re.sub(r'\*\*(.+?)\*\*', r'<strong>\1</strong>', html)
        # Lists
        html = re.sub(r'^- (.+)$', r'<li>\1</li>', html, flags=re.MULTILINE)
        html = re.sub(r'(<li>.*</li>\n)+', r'<ul>\g<0></ul>', html)
        # Paragraphs
        html = re.sub(r'\n\n(.+?)\n\n', r'\n<p>\1</p>\n', html)
        return html
    
    def write_outputs(self):
        """Write all output formats"""
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        
        # Markdown
        md = self.generate_markdown()
        (OUTPUT_DIR / "icd.md").write_text(md)
        print(f"Generated: {OUTPUT_DIR / 'icd.md'}")
        
        # HTML
        html = self.generate_html()
        (OUTPUT_DIR / "icd.html").write_text(html)
        print(f"Generated: {OUTPUT_DIR / 'icd.html'}")
        
        # JSON (machine-readable)
        json_data = {
            'generated_at': datetime.now().isoformat(),
            'version': '1.0.0',
            'messages': self.messages,
            'services': self.services,
            'enums': self.enums
        }
        (OUTPUT_DIR / "icd.json").write_text(json.dumps(json_data, indent=2))
        print(f"Generated: {OUTPUT_DIR / 'icd.json'}")

def main():
    generator = ICDGenerator()
    generator.load_contracts()
    generator.write_outputs()
    print("ICD generation complete!")

if __name__ == "__main__":
    main()