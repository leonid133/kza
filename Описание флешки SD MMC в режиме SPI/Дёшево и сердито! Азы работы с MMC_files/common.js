function ShowHideElement(elementID)
{
    if (!document.getElementById(elementID))
        return false;
        
    if (document.getElementById(elementID).style.display=='none')
    {
        document.getElementById(elementID).style.display='';
    }
    else
    {
        document.getElementById(elementID).style.display='none';
    }
}

function openPopup(href,w,h)
{
    var left = (self.opera ? iWidth : screen.availWidth)/2 - w/2;
    var top =  (self.opera ? iHeight : screen.availHeight)/2 - h/2;
    var w=window.open(href,"_blank",'width='+w+',height='+h+',left='+left+',top='+top+',screenX=0,screenY=0,location=no,toolbar=no,menubar=no,status=no,scrollbars=yes,resizable=yes');
    w.focus();
}


function rand()
{
    var now=new Date()
    var num=(now.getSeconds())%10
    var num=num+1
    return num;
}

function refreshFormCode()
{
    if (document.getElementById('form_code_image'))
    {
        document.getElementById('form_code_image').src="/project/form_code.php?hash="+rand();
    }
}
