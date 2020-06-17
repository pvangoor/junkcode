import requests
import urllib
from bs4 import BeautifulSoup

url="https://www.thebiomics.com/blog/springer-free-books-download.html?fbclid=IwAR2YRaJyUMEBkstYZrXPLx4iActO7TTwvepSYBJzAAAxhJji1PmVLpdRj_M"

req = urllib.request.Request(url,  headers={'User-Agent' : "Magic Browser"})
html = urllib.request.urlopen( req )
soup = BeautifulSoup(html)
tags = soup.findAll("a", {"class": "download-link link-external link m-0"})

num = len(tags)
count = 0
for tag in tags:
    dl_url = tag.get("download")
    
    print("Downloading:\n{}".format(dl_url))

    pre = "https://link.springer.com/content/pdf/"
    title = dl_url[len(pre):]
    r = requests.get(dl_url, stream=True)
    with open("pdfs/"+title, 'wb') as f:
        f.write(r.content)

    count += 1
    print("Downloaded {}/{}".format(count, num))



# r = requests.get(url, stream=True)
# with open('C:/Users/MICRO HARD/myfile.pdf', 'wb') as f:
    # f.write(r.content)

# <a class="download-link link-external link m-0" download="https://link.springer.com/content/pdf/10.1007%2F978-0-306-48048-5.pdf" href="https://link.springer.com/content/pdf/10.1007%2F978-0-306-48048-5.pdf" rel="" target="_blank" title="Download Link"><span class="icon-download">&nbsp;</span>Download Link</a>